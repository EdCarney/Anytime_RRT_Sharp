#include "ArrtsParams.hpp"
#include "ArrtsService.hpp"

int main(int argc, char** argv)
{
    ArrtsService service;

    if (argc > 1)
        for (int i = 1; i < argc; ++i)
            service.calculatePath(ArrtsParams(argv[i]), argv[i]);
    else
        service.calculatePath(ArrtsParams("./test"), "./test");

    return 0;
}