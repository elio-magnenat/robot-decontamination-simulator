#include "io.h"

using namespace std;

State OpenJSON_State(const string& filename, bool* outOk) {
    if (outOk != nullptr) *outOk = false;

    if (filename.empty())
        return {};

    ifstream ifs(filename);
    if (!ifs) {
        cerr << "Erreur : fichier state invalide" << endl;
        return {};
    }

    State state;
    try {
        ifs >> state;
    }
    catch (exception const& e) {
        cerr << "Fichier state invalide" << endl;
        return {};
    }

    //The JSON contract needs RobotInfo::angle in [0,360[ ; the given plans
    //can hold negative angles (-90 in validation 05).
    for (RobotInfo& robot : state.robots)
        robot.angle = normalizeDeg360(robot.angle);

    if (outOk != nullptr) *outOk = true;
    return state;
}


Constraint OpenJSON_Constraint(const string& filename, bool* outOk) {
    if (outOk != nullptr) *outOk = false;

    if (filename.empty())
        return {};

    ifstream ifs(filename);
    if (!ifs) {
        cerr << "Erreur : fichier constraint invalide" << endl;
        return {};
    }

    Constraint constraint;
    try {
        ifs >> constraint;
    }
    catch (exception const& e) {
        cerr << "Fichier constraint invalide" << endl;
        return {};
    }

    if (outOk != nullptr) *outOk = true;
    return constraint;
}

/**
 * @brief File name without its folder and its extension : "../Data/05.stat" -> "05"
 *
 * @param path : path of the file
 * @return only the name of the file
 */
static string extractFilename(const string& path) {
    string::size_type slash = path.find_last_of("/\\");
    string::size_type dot = 0;
    string stem = path;

    if (slash != string::npos)
        stem = stem.substr(slash + 1);

    dot = stem.find_last_of('.');
    if (dot != string::npos && dot != 0)
        stem = stem.substr(0, dot);

    return stem;
}

string buildTimelineName(const string& stateFile, const string& constraintFile) {
    string::size_type slash = stateFile.find_last_of("/\\");
    string directory = "";

    //the timeline is written next to the state that produced it
    if (slash != string::npos)
        directory = stateFile.substr(0, slash + 1);

    return directory + extractFilename(stateFile) + "_" + extractFilename(constraintFile)
        + "_" + GROUP_ID + ".tlin";
}

int exportTimeline(const string& filename, TimeLine& timeline) {
    if (filename.empty())
        return ERROR::EMPTY_NAME_FILE;

    ofstream ofs(filename);
    if (!ofs) {
        cerr << "Erreur : exportation de la timeline" << endl;
        return ERROR::UNAVAILABLE_WRITING;
    }
    try {
        ofs << timeline;
    }
    catch (exception const& e) {
        cerr << "Fichier d'exportation failed" << endl;
        return ERROR::INVALID_FILE;
    }
    return ERROR::SUCCESS;
}
