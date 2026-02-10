/**
 * Copyright © 2021 IBM Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "config.h"

#include "shutdown_alarm_monitor.hpp"

#include <unistd.h>

#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Logging/Entry/server.hpp>

#include <format>
#include <sstream>

namespace sensor::monitor
{
using namespace phosphor::logging;
using namespace phosphor::fan::util;
using namespace phosphor::fan;
namespace fs = std::filesystem;

const std::map<ShutdownType, std::string> shutdownInterfaces{
    {ShutdownType::hard, "xyz.openbmc_project.Sensor.Threshold.HardShutdown"},
    {ShutdownType::soft, "xyz.openbmc_project.Sensor.Threshold.SoftShutdown"},
    {ShutdownType::leak, "xyz.openbmc_project.State.LeakDetector"}};

const std::map<ShutdownType, std::map<AlarmType, std::string>> alarmProperties{
    {ShutdownType::hard,
     {{AlarmType::low, "HardShutdownAlarmLow"},
      {AlarmType::high, "HardShutdownAlarmHigh"}}},
    {ShutdownType::soft,
     {{AlarmType::low, "SoftShutdownAlarmLow"},
      {AlarmType::high, "SoftShutdownAlarmHigh"}}},
    {ShutdownType::leak, {{AlarmType::high, "DetectorState"}}}};

const std::map<ShutdownType, std::chrono::milliseconds> shutdownDelays{
    {ShutdownType::hard,
     std::chrono::milliseconds{SHUTDOWN_ALARM_HARD_SHUTDOWN_DELAY_MS}},
    {ShutdownType::soft,
     std::chrono::milliseconds{SHUTDOWN_ALARM_SOFT_SHUTDOWN_DELAY_MS}},
    {ShutdownType::leak, std::chrono::milliseconds{0}}};

const std::map<ShutdownType, std::map<AlarmType, std::string>> alarmEventLogs{
    {ShutdownType::hard,
     {{AlarmType::high,
       "xyz.openbmc_project.Sensor.Threshold.Error.HardShutdownAlarmHigh"},
      {AlarmType::low, "xyz.openbmc_project.Sensor.Threshold.Error."
                       "HardShutdownAlarmLow"}}},
    {ShutdownType::soft,
     {{AlarmType::high,
       "xyz.openbmc_project.Sensor.Threshold.Error.SoftShutdownAlarmHigh"},
      {AlarmType::low, "xyz.openbmc_project.Sensor.Threshold.Error."
                       "SoftShutdownAlarmLow"}}},
    {ShutdownType::leak,
     {{AlarmType::high, "xyz.openbmc_project.State.LeakDetector.Error."
                        "LeakDetectedCritical"}}}};

const std::map<ShutdownType, std::map<AlarmType, std::string>>
    alarmClearEventLogs{
        {ShutdownType::hard,
         {{AlarmType::high, "xyz.openbmc_project.Sensor.Threshold.Error."
                            "HardShutdownAlarmHighClear"},
          {AlarmType::low, "xyz.openbmc_project.Sensor.Threshold.Error."
                           "HardShutdownAlarmLowClear"}}},
        {ShutdownType::soft,
         {{AlarmType::high, "xyz.openbmc_project.Sensor.Threshold.Error."
                            "SoftShutdownAlarmHighClear"},
          {AlarmType::low, "xyz.openbmc_project.Sensor.Threshold.Error."
                           "SoftShutdownAlarmLowClear"}}},
        {ShutdownType::leak,
         {{AlarmType::high, "xyz.openbmc_project.State.LeakDetector.Error."
                            "LeakDetectedNormal"}}}};

constexpr auto valueInterface = "xyz.openbmc_project.Sensor.Value";
constexpr auto valueProperty = "Value";
constexpr auto leakConfigInterface =
    "xyz.openbmc_project.Configuration.LeakDetectionPolicy";
constexpr auto leakDetectorNameProp = "LeakDetectorName";
constexpr auto leakCriticalReactionProp = "CriticalReactionType";
constexpr auto leakReactionDelayProp = "ReactionDelaySeconds";
constexpr auto leakCriticalState =
    "xyz.openbmc_project.State.LeakDetector.DetectorStateEnum.Critical";
constexpr auto chassisStateService = "xyz.openbmc_project.State.Chassis";
constexpr auto chassisStatePath = "/xyz/openbmc_project/state/chassis0";
constexpr auto chassisStateInterface = "xyz.openbmc_project.State.Chassis";
constexpr auto chassisStateTransitionProp = "RequestedPowerTransition";
constexpr auto chassisStateOff =
    "xyz.openbmc_project.State.Chassis.Transition.Off";

constexpr auto hostStateService = "xyz.openbmc_project.State.Host";
constexpr auto hostStatePath = "/xyz/openbmc_project/state/host0";
constexpr auto hostStateInterface = "xyz.openbmc_project.State.Host";
constexpr auto hostStateTransitionProp = "RequestedHostTransition";
constexpr auto hostStateOff = "xyz.openbmc_project.State.Host.Transition.Off";

const auto loggingService = "xyz.openbmc_project.Logging";
const auto loggingPath = "/xyz/openbmc_project/logging";
const auto loggingCreateIface = "xyz.openbmc_project.Logging.Create";

using namespace sdbusplus::bus::match;

ShutdownAlarmMonitor::ShutdownAlarmMonitor(
    sdbusplus::bus_t& bus, sdeventplus::Event& event,
    std::shared_ptr<PowerState> powerState) :
    bus(bus), event(event), _powerState(std::move(powerState)),
    hardShutdownMatch(bus,
                      "type='signal',member='PropertiesChanged',"
                      "path_namespace='/xyz/openbmc_project/sensors',"
                      "arg0='" +
                          shutdownInterfaces.at(ShutdownType::hard) + "'",
                      std::bind(&ShutdownAlarmMonitor::propertiesChanged, this,
                                std::placeholders::_1)),
    softShutdownMatch(bus,
                      "type='signal',member='PropertiesChanged',"
                      "path_namespace='/xyz/openbmc_project/sensors',"
                      "arg0='" +
                          shutdownInterfaces.at(ShutdownType::soft) + "'",
                      std::bind(&ShutdownAlarmMonitor::propertiesChanged, this,
                                std::placeholders::_1)),
    leakDetectorMatch(bus,
                      "type='signal',member='PropertiesChanged',"
                      "path_namespace='/xyz/openbmc_project/state',"
                      "arg0='" +
                          shutdownInterfaces.at(ShutdownType::leak) + "'",
                      std::bind(&ShutdownAlarmMonitor::propertiesChanged, this,
                                std::placeholders::_1))
{
    _powerState->addCallback("shutdownMon",
                             std::bind(&ShutdownAlarmMonitor::powerStateChanged,
                                       this, std::placeholders::_1));
    findAlarms();

    if (_powerState->isPowerOn())
    {
        checkAlarms();

        // Get rid of any previous saved timestamps that don't
        // apply anymore.
        timestamps.prune(alarms);
    }
    else
    {
        timestamps.clear();
    }
}

void ShutdownAlarmMonitor::findAlarms()
{
    // Find all shutdown threshold ifaces currently on D-Bus.
    for (const auto& [shutdownType, interface] : shutdownInterfaces)
    {
        auto paths = SDBusPlus::getSubTreePathsRaw(bus, "/", interface, 0);

        auto shutdownType2 = shutdownType;

        std::for_each(
            paths.begin(), paths.end(),
            [this, shutdownType2](const auto& path) {
                alarms.emplace(AlarmKey{path, shutdownType2, AlarmType::high},
                               nullptr);
                if (shutdownType2 != ShutdownType::leak)
                {
                    alarms.emplace(
                        AlarmKey{path, shutdownType2, AlarmType::low}, nullptr);
                }
            });
    }
}

void ShutdownAlarmMonitor::checkAlarms()
{
    for (auto& [alarmKey, timer] : alarms)
    {
        const auto& [sensorPath, shutdownType, alarmType] = alarmKey;
        const auto& interface = shutdownInterfaces.at(shutdownType);
        auto propertyName = alarmProperties.at(shutdownType).at(alarmType);
        bool value = false;

        try
        {
            if (shutdownType == ShutdownType::leak)
            {
                auto state = SDBusPlus::getProperty<std::string>(
                    bus, sensorPath, interface, propertyName);
                if (state == leakCriticalState)
                {
                    value = true;
                }
            }
            else
            {
                value = SDBusPlus::getProperty<bool>(bus, sensorPath, interface,
                                                     propertyName);
            }
        }
        catch (const std::exception& e)
        {
            // The sensor isn't on D-Bus anymore
            log<level::INFO>(std::format("No {} interface on {} anymore.",
                                         interface, sensorPath)
                                 .c_str());
            continue;
        }

        checkAlarm(value, alarmKey);
    }
}

void ShutdownAlarmMonitor::propertiesChanged(sdbusplus::message_t& message)
{
    std::map<std::string, std::variant<bool, std::string>> properties;
    std::string interface;

    if (!_powerState->isPowerOn())
    {
        return;
    }

    message.read(interface, properties);

    auto type = getShutdownType(interface);
    if (!type)
    {
        return;
    }

    std::string sensorPath = message.get_path();

    if (*type == ShutdownType::leak)
    {
        const auto& alarmName = alarmProperties.at(*type).at(AlarmType::high);
        if (properties.count(alarmName) > 0)
        {
            AlarmKey alarmKey{sensorPath, *type, AlarmType::high};
            auto alarm = alarms.find(alarmKey);
            if (alarm == alarms.end())
            {
                alarms.emplace(alarmKey, nullptr);
            }
            auto state = std::get<std::string>(properties.at(alarmName));
            bool value = (state == leakCriticalState);

            checkAlarm(value, alarmKey);
        }
        return;
    }

    const auto& lowAlarmName = alarmProperties.at(*type).at(AlarmType::low);
    if (properties.count(lowAlarmName) > 0)
    {
        AlarmKey alarmKey{sensorPath, *type, AlarmType::low};
        auto alarm = alarms.find(alarmKey);
        if (alarm == alarms.end())
        {
            alarms.emplace(alarmKey, nullptr);
        }
        checkAlarm(std::get<bool>(properties.at(lowAlarmName)), alarmKey);
    }

    const auto& highAlarmName = alarmProperties.at(*type).at(AlarmType::high);
    if (properties.count(highAlarmName) > 0)
    {
        AlarmKey alarmKey{sensorPath, *type, AlarmType::high};
        auto alarm = alarms.find(alarmKey);
        if (alarm == alarms.end())
        {
            alarms.emplace(alarmKey, nullptr);
        }
        checkAlarm(std::get<bool>(properties.at(highAlarmName)), alarmKey);
    }
}

void ShutdownAlarmMonitor::checkAlarm(bool value, const AlarmKey& alarmKey)
{
    auto alarm = alarms.find(alarmKey);
    if (alarm == alarms.end())
    {
        return;
    }

    // Start or stop the timer if necessary.
    auto& timer = alarm->second;
    if (value)
    {
        if (!timer)
        {
            startTimer(alarmKey);
        }
    }
    else
    {
        if (timer)
        {
            stopTimer(alarmKey);
        }
    }
}

void ShutdownAlarmMonitor::startTimer(const AlarmKey& alarmKey)
{
    const auto& [sensorPath, shutdownType, alarmType] = alarmKey;
    const auto& propertyName = alarmProperties.at(shutdownType).at(alarmType);
    std::chrono::milliseconds shutdownDelay{shutdownDelays.at(shutdownType)};
    std::optional<double> value;

    if (shutdownType == ShutdownType::leak)
    {
        auto [reaction, delay] = getLeakConfig(sensorPath);
        log<level::INFO>(
            std::format("Leak detector {} config: reaction={}, delay={}",
                        sensorPath, reaction, delay)
                .c_str());

        if (reaction != "ForceOff")
        {
            return;
        }
        shutdownDelay = std::chrono::seconds(delay);
    }

    auto alarm = alarms.find(alarmKey);
    if (alarm == alarms.end())
    {
        throw std::runtime_error("Couldn't find alarm inside startTimer");
    }

    if (shutdownType != ShutdownType::leak)
    {
        try
        {
            value = SDBusPlus::getProperty<double>(
                bus, sensorPath, valueInterface, valueProperty);
        }
        catch (const DBusServiceError& e)
        {
            // If the sensor was just added, the Value interface for it may
            // not be in the mapper yet.  This could only happen if the sensor
            // application was started with power up and the value exceeded the
            // threshold immediately.
        }
    }

    createEventLog(alarmKey, true, value);

    uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();

    // If there is a saved timestamp for this timer, then we were restarted
    // while the timer was running.  Calculate the remaining time to use
    // for the timer.
    auto previousStartTime = timestamps.get().find(alarmKey);
    if (previousStartTime != timestamps.get().end())
    {
        const uint64_t& original = previousStartTime->second;

        log<level::INFO>(std::format("Found previously running {} timer "
                                     "for {} with start time {}",
                                     propertyName, sensorPath, original)
                             .c_str());

        // Sanity check it isn't total garbage.
        if (now > original)
        {
            uint64_t remainingTime = 0;
            auto elapsedTime = now - original;

            if (elapsedTime < static_cast<uint64_t>(shutdownDelay.count()))
            {
                remainingTime = static_cast<uint64_t>(shutdownDelay.count()) -
                                elapsedTime;
            }

            shutdownDelay = std::chrono::milliseconds{remainingTime};
        }
        else
        {
            log<level::WARNING>(
                std::format(
                    "Restarting {} shutdown timer for {} for full "
                    "time because saved time {} is after current time {}",
                    propertyName, sensorPath, original, now)
                    .c_str());
        }
    }

    if (value)
    {
        log<level::INFO>(
            std::format(
                "Starting {}ms {} shutdown timer due to sensor {} value {}",
                shutdownDelay.count(), propertyName, sensorPath, *value)
                .c_str());
    }
    else
    {
        log<level::INFO>(
            std::format("Starting {}ms {} shutdown timer due to sensor {}",
                        shutdownDelay.count(), propertyName, sensorPath)
                .c_str());
    }

    auto& timer = alarm->second;

    timer = std::make_unique<
        sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>>(
        event, std::bind(&ShutdownAlarmMonitor::timerExpired, this, alarmKey));

    timer->restartOnce(shutdownDelay);

    // Note that if this key is already in the timestamps map because
    // the timer was already running the timestamp wil not be updated.
    timestamps.add(alarmKey, now);
}

void ShutdownAlarmMonitor::stopTimer(const AlarmKey& alarmKey)
{
    const auto& [sensorPath, shutdownType, alarmType] = alarmKey;
    const auto& propertyName = alarmProperties.at(shutdownType).at(alarmType);

    std::optional<double> value;
    if (shutdownType != ShutdownType::leak)
    {
        value = SDBusPlus::getProperty<double>(bus, sensorPath, valueInterface,
                                               valueProperty);
    }

    auto alarm = alarms.find(alarmKey);
    if (alarm == alarms.end())
    {
        throw std::runtime_error("Couldn't find alarm inside stopTimer");
    }

    createEventLog(alarmKey, false, value);

    if (value)
    {
        log<level::INFO>(
            std::format("Stopping {} shutdown timer due to sensor {} value {}",
                        propertyName, sensorPath, *value)
                .c_str());
    }
    else
    {
        log<level::INFO>(std::format("Stopping {} shutdown timer for sensor {}",
                                     propertyName, sensorPath)
                             .c_str());
    }

    auto& timer = alarm->second;
    timer->setEnabled(false);
    timer.reset();

    timestamps.erase(alarmKey);
}

void ShutdownAlarmMonitor::createBmcDump() const
{
    try
    {
        util::SDBusPlus::callMethod(
            "xyz.openbmc_project.Dump.Manager", "/xyz/openbmc_project/dump/bmc",
            "xyz.openbmc_project.Dump.Create", "CreateDump",
            std::vector<
                std::pair<std::string, std::variant<std::string, uint64_t>>>());
    }
    catch (const std::exception& e)
    {
        auto message = std::format(
            "Caught exception while creating BMC dump: {}", e.what());

        log<level::ERR>(message.c_str());
    }
}

void ShutdownAlarmMonitor::timerExpired(const AlarmKey& alarmKey)
{
    const auto& [sensorPath, shutdownType, alarmType] = alarmKey;
    const auto& propertyName = alarmProperties.at(shutdownType).at(alarmType);

    std::optional<double> value;
    if (shutdownType != ShutdownType::leak)
    {
        value = SDBusPlus::getProperty<double>(bus, sensorPath, valueInterface,
                                               valueProperty);
    }

    log<level::ERR>(
        std::format(
            "The {} shutdown timer expired for sensor {}, shutting down",
            propertyName, sensorPath)
            .c_str());

    // Re-send the event log.  If someone didn't want this it could be
    // wrapped by a compile option.
    createEventLog(alarmKey, true, value, true);

    if (shutdownType == ShutdownType::hard ||
        shutdownType == ShutdownType::leak)
    {
        try
        {
            SDBusPlus::setProperty(bus, chassisStateService, chassisStatePath,
                                   chassisStateInterface,
                                   chassisStateTransitionProp,
                                   std::string(chassisStateOff));
        }
        catch (const std::exception& e)
        {
            log<level::ERR>(
                std::format("Failed to trigger hard shutdown: {}", e.what())
                    .c_str());
        }
    }
    else
    {
        try
        {
            SDBusPlus::setProperty(bus, hostStateService, hostStatePath,
                                   hostStateInterface, hostStateTransitionProp,
                                   std::string(hostStateOff));
        }
        catch (const std::exception& e)
        {
            log<level::ERR>(
                std::format("Failed to trigger soft shutdown: {}", e.what())
                    .c_str());
        }
    }

    timestamps.erase(alarmKey);
    createBmcDump();
}

void ShutdownAlarmMonitor::powerStateChanged(bool powerStateOn)
{
    if (powerStateOn)
    {
        checkAlarms();
    }
    else
    {
        timestamps.clear();

        // Cancel and delete all timers
        std::for_each(alarms.begin(), alarms.end(), [](auto& alarm) {
            auto& timer = alarm.second;
            if (timer)
            {
                timer->setEnabled(false);
                timer.reset();
            }
        });
    }
}

// Structure: {shutdown type} shutdown timer {started/stopped/expired} for
// sensor {sensor} [at value {value}]. [Performing {shutdown type}
// shutdown.][Shutdown cancelled.]
void ShutdownAlarmMonitor::createEventLog(
    const AlarmKey& alarmKey, bool alarmValue,
    const std::optional<double>& sensorValue, bool isPowerOffError)
{
    using namespace sdbusplus::xyz::openbmc_project::Logging::server;
    const auto& [sensorPath, shutdownType, alarmType] = alarmKey;
    std::map<std::string, std::string> ad{{"SENSOR_NAME", sensorPath},
                                          {"_PID", std::to_string(getpid())}};

    std::stringstream sensorPathStream(sensorPath);
    std::string part;
    std::vector<std::string> sensorPathVect;

    while (std::getline(sensorPathStream, part, '/'))
    {
        sensorPathVect.push_back(part);
    }
    std::string sensorName = sensorPathVect.back();

    // Tell user which kind of timer this is
    std::string errorMessage = "";
    switch (shutdownType)
    {
        case ShutdownType::hard:
            errorMessage += "Hard shutdown timer ";
            break;
        case ShutdownType::soft:
            errorMessage += "Soft shutdown timer ";
            break;
        case ShutdownType::leak:
            errorMessage += "Leak detector shutdown timer ";
            break;
    }
    // Timer expired (shutdown occurring)
    if (alarmValue && isPowerOffError)
    {
        errorMessage += "expired for sensor " + sensorName;
    }
    // Timer started
    else if (alarmValue)
    {
        errorMessage += "started for sensor " + sensorName;
    }
    // Timer stopped before shutdown
    else
    {
        errorMessage += "stopped for sensor " + sensorName;
    }
    if (sensorValue)
    {
        errorMessage += " at value " + std::to_string(*sensorValue);
    }
    errorMessage += ". ";
    if (alarmValue && isPowerOffError)
    {
        switch (shutdownType)
        {
            case ShutdownType::hard:
                errorMessage += "Performing hard shutdown.";
                break;
            case ShutdownType::soft:
                errorMessage += "Performing soft shutdown.";
                break;
            case ShutdownType::leak:
                errorMessage += "Performing leak detector shutdown.";
                break;
        }
    }
    else if (!alarmValue && !isPowerOffError)
    {
        errorMessage += " Shutdown cancelled.";
    }

    // severity = Critical if a power off
    // severity = Error if alarm was asserted
    // severity = Informational if alarm was deasserted
    Entry::Level severity = Entry::Level::Error;
    if (isPowerOffError)
    {
        severity = Entry::Level::Critical;
    }
    else if (!alarmValue)
    {
        severity = Entry::Level::Informational;
    }

    if (sensorValue)
    {
        ad.emplace("SENSOR_VALUE", std::to_string(*sensorValue));
    }

    // If this is a power off, specify that it's a power
    // fault and a system termination.  This is used by some
    // implementations for service reasons.
    if (isPowerOffError)
    {
        ad.emplace("SEVERITY_DETAIL", "SYSTEM_TERM");
    }

    SDBusPlus::callMethod(loggingService, loggingPath, loggingCreateIface,
                          "Create", errorMessage, convertForMessage(severity),
                          ad);
}

std::optional<ShutdownType> ShutdownAlarmMonitor::getShutdownType(
    const std::string& interface) const
{
    auto it = std::find_if(
        shutdownInterfaces.begin(), shutdownInterfaces.end(),
        [interface](const auto& a) { return a.second == interface; });

    if (it == shutdownInterfaces.end())
    {
        return std::nullopt;
    }

    return it->first;
}

std::pair<std::string, uint64_t> ShutdownAlarmMonitor::getLeakConfig(
    const std::string& detectorPath)
{
    std::string path = detectorPath;
    std::string reaction;
    std::string foundInterface;

    // Try to find it in the subtree
    try
    {
        auto paths = SDBusPlus::getSubTreePathsRaw(
            bus, "/xyz/openbmc_project/inventory", leakConfigInterface, 0);

        // Find the path that ends with the sensor name
        std::string sensorName = std::filesystem::path(detectorPath).filename();

        auto it = std::find_if(
            paths.begin(), paths.end(), [&sensorName, this](const auto& p) {
                try
                {
                    auto name = SDBusPlus::getProperty<std::string>(
                        bus, p, leakConfigInterface, leakDetectorNameProp);
                    return name == sensorName;
                }
                catch (...)
                {
                    return false;
                }
            });

        if (it != paths.end())
        {
            path = *it;
            reaction = SDBusPlus::getProperty<std::string>(
                bus, path, leakConfigInterface, leakCriticalReactionProp);
            log<level::INFO>(
                std::format("Found leak config for {} at {} on interface {}",
                            detectorPath, path, leakConfigInterface)
                    .c_str());
        }
    }
    catch (...)
    {
        log<level::ERR>(
            std::format(
                "Failed to get valid leak detector path for {} from interface {}",
                detectorPath, leakConfigInterface)
                .c_str());
    }

    if (reaction.empty())
    {
        return {"", 0};
    }

    uint64_t delay = 0;
    try
    {
        double d = SDBusPlus::getProperty<double>(
            bus, path, leakConfigInterface, leakReactionDelayProp);
        delay = static_cast<uint64_t>(d);
    }
    catch (...)
    {
        try
        {
            delay = SDBusPlus::getProperty<uint64_t>(
                bus, path, leakConfigInterface, leakReactionDelayProp);
        }
        catch (...)
        {}
    }

    return {reaction, delay};
}

} // namespace sensor::monitor
