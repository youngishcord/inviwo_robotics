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
#pragma once

#include <inviwo/robotics/roboticsmoduledefine.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/properties/buttonproperty.h>
#include <inviwo/core/processors/processorinfo.h>
#include <inviwo/core/properties/stringproperty.h>
#include <inviwo/core/properties/compositeproperty.h>
#include <inviwo/core/properties/listproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>

namespace inviwo {

/**
 * \brief VERY_BRIEFLY_DESCRIBE_THE_CLASS
 * DESCRIBE_THE_CLASS_FROM_A_DEVELOPER_PERSPECTIVE
 */
class IVW_MODULE_ROBOTICS_API InverseKinematicProcessor : public Processor {
public:
    InverseKinematicProcessor();
    virtual ~InverseKinematicProcessor() = default;

    virtual void process() override;

    virtual const ProcessorInfo& getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    void forwardRecalc();

    float angCalc(vec3 a, vec3 b, vec3 n);

private:

    CompositeProperty composite;
    ListProperty list;

    //FloatMat4Property g1;
    //FloatMat4Property g2;

    bool onInverse = false;
    bool onForward = true;
    
    FloatMat4Property startMat;
    
    FloatProperty g1;
    FloatProperty l1;
    FloatMat4Property g1Mat;
    FloatVec3Property outG1Pos;

    FloatProperty g2;
    FloatProperty l2;
    FloatMat4Property g2Mat;
    FloatVec3Property outG2Pos;
    
    FloatProperty g3;
    FloatProperty l3;
    FloatMat4Property g3Mat;
    FloatVec3Property outG3Pos;

    FloatProperty g4;
    FloatProperty l4;
    FloatMat4Property g4Mat;
    FloatVec3Property outG4Pos;
    
    FloatProperty g5;
    FloatProperty l5;
    FloatMat4Property g5Mat;
    FloatVec3Property outG5Pos;

    FloatProperty g6;
    FloatProperty l6;
    FloatMat4Property g6Mat;
    FloatVec3Property outG6Pos;

    FloatMat4Property FMat;
    FloatVec3Property outFPos;

    FloatMat4Property target;
};


}  // namespace inviwo