#include <string>
#include "ArrtsParams.hpp"
#include "ArrtsService.hpp"
#include "ManeuverEngine.hpp"

using namespace std;

string getOutputDir(char* dir, ManeuverType maneuverType)
{
    string dirStr(dir);
    if (maneuverType == Dubins3d)
        return dirStr + "_Dubins3d";
    return dirStr + "_DirectPath";
}

ManeuverType getManeuverType(char* maneuver)
{
    string maneuveStr(maneuver);
    if (maneuveStr == "Dubins3d")
        return Dubins3d;
    else if (maneuveStr != "DirectPath")
        printf("WARNING: Unrecognized or unspecified maneuver type, defaulting to DirectPath...\n");
    return DirectPath;
}

int main(int argc, char** argv)
{
    ArrtsService service;
    ManeuverType maneuverType = getManeuverType(argv[argc - 1]);

    if (argc > 1)
        for (int i = 1; i < argc; ++i)
            service.calculatePath(ArrtsParams(argv[i]), getOutputDir(argv[i], maneuverType), maneuverType);
    else
        service.calculatePath(ArrtsParams("./test"), "./test", maneuverType);

    return 0;
}