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

#include <inviwo/robotics/processors/roboticsProcessor.h>

namespace inviwo {

    const ProcessorInfo RoboticsProcessor::processorInfo_{
        "org.robotics.processor",  // Class identifier
        "roboticsProcessor",                    // Display name
        "robotics",                     // Category
        CodeState::Stable,                   // Code state
        Tags::CPU                           // Tags
        };

    RoboticsProcessor::RoboticsProcessor()
        : Processor()
        , composite("composite", "Composite")
        , list("list", "List")
        , g1("g1", "g1")
        , g2("g2", "g2")
        , l1("l1", "l1")
        , l2("l2", "l2")
        , startPos("startPos", "startPos", (1.0f))
        , resultPos("resultPos", "resultPos", (1.0f))
        , outFPos("outFPos", "outFPos")
        , g1Pos("g1Pos", "g1Pos", (1.0f))
        , outG1Pos("outG1Pos", "outG1Pos")
        , g2Pos("g2Pos", "g2Pos", (1.0f))
        , outG2Pos("outG2Pos", "outG2Pos")
        , g3("g3", "g3")
        , l3("l3", "l3")
        , g3Pos("g3Pos", "g3Pos", (1.0f))
        , outG3Pos("outG3Pos", "outG3Pos")
    {
        addProperties(composite, list, g1, g2, l1, l2,
            startPos, resultPos, g1Pos, g2Pos,
            outG1Pos, outG2Pos, outFPos

            , g3, l3, g3Pos, outG3Pos
        );

        outFPos.onChange([this]() {
            if (!onForward) {
                calcInverse();
            }
            });

        LogInfo("robotics loaded");
    }

void inviwo::RoboticsProcessor::process() {
    //LogInfo("ECHO FROM FIRST PROCESSOR");

    if (!onInverse) {
        onForward = true;
        g1Pos = glm::rotate(startPos.get(), glm::radians(g1.get()), glm::vec3(0, 0, 1));
        outG1Pos = glm::vec3(g1Pos.get()[3]);

        g2Pos = glm::rotate(
            glm::translate(g1Pos.get(), glm::vec3(0.0f, 0.0f, l1.get())),
            glm::radians(g2.get()), glm::vec3(0, 1, 0)
        );
        outG2Pos = glm::vec3(g2Pos.get()[3]);

        g3Pos = glm::rotate(
            glm::translate(g2Pos.get(), glm::vec3(0.0f, 0.0f, l2.get())),
            glm::radians(g3.get()), glm::vec3(0, 1, 0)
        );
        outG3Pos = glm::vec3(g3Pos.get()[3]);

        resultPos = glm::translate(g3Pos.get(), glm::vec3(0.0f, 0.0f, l3.get()));
        if (!onInverse) {
            outFPos = glm::vec3(resultPos.get()[3]);
        }
    }
        //g1Pos = glm::rotate(startPos.get(), g1.get(), glm::vec3(0.0f, 0.0f, 1.0f));
    //}
    onForward = false;
    onInverse = false;
}

void RoboticsProcessor::calcInverse() {
    onInverse = true;

    //LogInfo(glm::degrees(glm::atan2(outFPos.get().y, outFPos.get().x)));
    g1.set(glm::degrees(glm::atan2(outFPos.get().y, outFPos.get().x)));

    vec3 temp = outFPos.get() - vec3(0.0f, 0.0f, l1.get());
    float psq = temp.y * temp.y + temp.x * temp.x;
    float q1 = glm::degrees(glm::atan2(temp.z, glm::sqrt(psq)));
    float bsq = temp.z*temp.z + temp.x*temp.x;
    float tt = (l2.get() * l2.get() + bsq - l3.get() * l3.get()) /
        (2 * l2.get() * glm::sqrt(bsq));
    //LogInfo(tt);
    float q2 = glm::degrees(glm::acos(
        tt
    ));
    //LogInfo(q2)
    g2.set(q2 + q1 + 180 + 90);

    tt = (l2.get() * l2.get() + l3.get() * l3.get() - bsq) /
        (2 * l2.get() * l3.get());
    //LogInfo(tt);
    float q3 = glm::degrees(glm::acos(
        tt
    ));
    g3.set(q3 + 90);

    onInverse = false;
    this->process();
}

const ProcessorInfo& RoboticsProcessor::getProcessorInfo() const
{
    return processorInfo_;
}

}  // namespace inviwo