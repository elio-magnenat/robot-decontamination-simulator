#ifndef PIN_2026_ALPINE_IO_H
#define PIN_2026_ALPINE_IO_H

#include <string>
#include <iostream>
#include <fstream>
#include "common/State.h"
#include "common/Constraint.h"
#include "common/Constants.h"
#include "common/angle_utils.h"
#include "common/TimeLine.h"
/**
 * @brief Opens a JSON file to get the state written inside
 *
 * @param filename : name of the file
 * @param outOk : set to true if the reading worked, false otherwise (can be nullptr)
 * @return the state written in the JSON file
 */
State OpenJSON_State(const std::string& filename, bool* outOk = nullptr);

/**
 * @brief Opens a JSON file to get the constraints written inside
 *
 * @param filename : name of the file
 * @param outOk : set to true if the reading worked, false otherwise (can be nullptr)
 * @return the constraints written in the JSON file
 */
Constraint OpenJSON_Constraint(const std::string& filename, bool* outOk = nullptr);

/**
 * @brief Builds the name of the timeline : <state>_<constraints>_<group>.tlin
 *
 * @param stateFile : path of the state file used
 * @param constraintFile : path of the constraints file used
 * @return the full path of the timeline to write
 */
std::string buildTimelineName(const std::string& stateFile,
                              const std::string& constraintFile);

/**
 * @brief Exports the generated timeLine
 *
 * @param filename : name of the file for the export
 * @param timeline : timeLine to export in .tlin format
* @return A status code (ex: ERROR::SUCCESS if everything went well)
 */
int exportTimeline(const std::string& filename, TimeLine& timeline);

#endif //PIN_2026_ALPINE_IO_H
