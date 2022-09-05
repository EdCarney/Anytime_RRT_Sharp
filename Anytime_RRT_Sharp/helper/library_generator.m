clear all

% set paths
currentFolder = pwd;
productPath = fullfile(pwd, "..");
baseLibPath = fullfile(productPath, "build");
baseHppPath = fullfile(productPath, "src");

% delete old versions
delete defineArrtsLibrary.*
if (isfile("ArrtsLibraryData.xml"))
    delete ArrtsLibraryData.xml
end
if (isfolder("ArrtsLibrary"))
    rmdir ArrtsLibrary s
end

% create library array
libs = ["libArrtsParams.a",...
    "libArrtsService.a",...
    "libConfigspaceGraph.a",...
    "libConfigspaceNode.a",...
    "libGeometry.a",...
    "libObstacle.a",...
    "libVehicle.a",...
    "libWorkspaceGraph.a"];

libPath = strings(size(libs));
for i = 1:size(libs, 2)
    libPath(i) = fullfile(baseLibPath, libs(i));
end

% set header file
hppPath = fullfile(baseHppPath, "ArrtsService.hpp");

% compile
libName = "ArrtsLibrary";
clibgen.generateLibraryDefinition(hppPath, Libraries=libPath, PackageName=libName)
build(defineArrtsLibrary)
addpath("ArrtsLibrary")