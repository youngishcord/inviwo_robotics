/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2024 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <inviwo/robotics/roboticsmodule.h>
#include <inviwo/robotics/processors/roboticsProcessor.h>

namespace inviwo {

RoboticsModule::RoboticsModule(InviwoApplication* app) : InviwoModule(app, "robotics") {
    LogInfo("robotics was loaded");
    // Add a directory to the search path of the Shadermanager
    // ShaderManager::getPtr()->addShaderSearchPath(getPath(ModulePath::GLSL));

    // Register objects that can be shared with the rest of inviwo here:

    // Processors
    registerProcessor<RoboticsProcessor>();

    // Properties
    // registerProperty<mySimpleTestModuleProperty>();

    // Readers and writes
    // registerDataReader(std::make_unique<mySimpleTestModuleReader>());
    // registerDataWriter(std::make_unique<mySimpleTestModuleWriter>());

    // Data converters
    // registerRepresentationConverter(std::make_unique<mySimpleTestModuleDisk2RAMConverter>());

    // Ports
    // registerPort<mySimpleTestModuleOutport>();
    // registerPort<mySimpleTestModuleInport>();

    // PropertyWidgets
    // registerPropertyWidget<mySimpleTestModulePropertyWidget, mySimpleTestModuleProperty>("Default");

    // Dialogs
    // registerDialog<mySimpleTestModuleDialog>(mySimpleTestModuleOutport);

    // Other things
    // registerCapabilities(std::make_unique<mySimpleTestModuleCapabilities>());
    // registerSettings(std::make_unique<mySimpleTestModuleSettings>());
    // registerMetaData(std::make_unique<mySimpleTestModuleMetaData>());
    // registerPortInspector("mySimpleTestModuleOutport", "path/workspace.inv");
    // registerProcessorWidget(std::string processorClassName, std::unique_ptr<ProcessorWidget> processorWidget); 
    // registerDrawer(std::make_unique_ptr<mySimpleTestModuleDrawer>());
}

}  // namespace inviwo