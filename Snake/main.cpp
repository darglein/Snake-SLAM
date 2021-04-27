/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "saiga/core/math/Eigen_Compile_Checker.h"
#include "saiga/core/util/FileSystem.h"
#include "saiga/core/util/commandLineArguments.h"
#include "saiga/core/util/env.h"

#include "System/System.h"

using namespace Snake;

#ifndef SAIGA_HAS_OMP
#    error NO OMP
#endif


int main(int argc, char** argv)
{
    OMP::setWaitPolicy(OMP::WaitPolicy::Passive);
    Saiga::EigenHelper::checkEigenCompabitilty<29364>();
    Saiga::EigenHelper::EigenCompileFlags flags;
    flags.create<23551>();
    std::cout << flags << std::endl;


    CLI::App app{"Snake SLAM", "snake"};

    // Define Options
    //  1. Create variables + their default values
    //  2. Use add_option to parse them from command line

    std::string config_file = "config_window.ini";
    std::string overrideDir = "";
    std::string name        = "";
    std::string outDir      = "";
    //    app.add
    app.add_option("config", config_file, "Config file.")->required();
    app.add_option("--dataset", overrideDir, "Optional. Overwrite config dataset directory.");
    app.add_option("--name", name, "Optional output name.");
    app.add_option("--outDir", outDir, "Optional output directory.");
    CLI11_PARSE(app, argc, argv);

    std::cout << "> Loading Config file " << config_file << std::endl;
    auto settings = Settings(config_file);
    settings.SetDefaultParametersForDataset();

    if (!overrideDir.empty())
    {
        std::cout << "Overriding dir with " << overrideDir << std::endl;
        settings.datasetParameters.dir = overrideDir;
    }

    if (!name.empty())
    {
        std::cout << "Overriding name with " << name << std::endl;
        settings.out_file_prefix = name;
    }

    if (!outDir.empty())
    {
        std::cout << "Overriding eval out with " << outDir << std::endl;
        settings.evalDir = outDir;
    }

    if (!std::filesystem::exists(settings.evalDir))
    {
        std::filesystem::create_directory(settings.evalDir);
    }
    if (!std::filesystem::exists(settings.evalDir + "/frames/"))
    {
        std::filesystem::create_directory(settings.evalDir + "/frames/");
    }

    Saiga::SaigaParameters sparams;
    sparams.fromConfigFile(config_file);
    Saiga::initSaiga(sparams);

    {
        Snake::System system(settings);
        system.run();
    }
    return 0;
}
