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

        , g4("g4", "g4")
        , l4("l4", "l4")
        , g4Pos("g4Pos", "g4Pos", (1.0f))
        , outG4Pos("outG4Pos", "outG4Pos")

        , g5("g5", "g5")
        , l5("l5", "l5")
        , g5Pos("g5Pos", "g5Pos", (1.0f))
        , outG5Pos("outG5Pos", "outG5Pos")

        , g6("g6", "g6")
        , l6("l6", "l6")
        , g6Pos("g6Pos", "g6Pos", (1.0f))
        , outG6Pos("outG6Pos", "outG6Pos")
    {
        addProperties(
            g1, g2, g3, g4, g5, g6,
            l1, l2, l3, l4, l5, l6,
            outG1Pos, outG2Pos, outG3Pos, outG4Pos, outG5Pos, outG6Pos, outFPos,
            composite, list, 
            startPos, resultPos, 
            g1Pos, g2Pos, g3Pos, g4Pos, g5Pos, g6Pos
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

        g4Pos = glm::rotate(
            glm::translate(g3Pos.get(), glm::vec3(0.0f, 0.0f, l3.get())),
            glm::radians(g4.get()), glm::vec3(0, 0, 1)
        );
        outG4Pos = glm::vec3(g4Pos.get()[3]);

        g5Pos = glm::rotate(
            glm::translate(g4Pos.get(), glm::vec3(0.0f, 0.0f, l4.get())),
            glm::radians(g5.get()), glm::vec3(0, 1, 0)
        );
        outG5Pos = glm::vec3(g5Pos.get()[3]);

        g6Pos = glm::rotate(
            glm::translate(g5Pos.get(), glm::vec3(0.0f, 0.0f, l5.get())),
            glm::radians(g6.get()), glm::vec3(0, 0, 1)
        );
        outG6Pos = glm::vec3(g6Pos.get()[3]);

        resultPos = glm::translate(g6Pos.get(), glm::vec3(0.0f, 0.0f, l6.get()));
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

    auto fifth = glm::vec3(outFPos.get() * vec3(0, 0, -l5.get() - l6.get()));

    float a1 = glm::pi<float>() + glm::atan2(fifth.y, fifth.x);
    g1.set(glm::degrees(a1));

    vec3 temp = fifth - vec3(0.0f, 0.0f, l1.get());

    //if (glm::length(temp) > (l2.get() + l3.get() + l4.get())) {
    //    onInverse = false;
    //    //this->process();
    //    return;
    //}
    //if (glm::length(temp) < glm::abs(l2.get() - l3.get() - l4.get())) {
    //    onInverse = false;
    //    //this->process();
    //    return;
    //}

    float forearm = l3.get() + l4.get();
    float psq = temp.y * temp.y + temp.x * temp.x; // Projection on the horizontal plane
    float q1 = glm::atan2(temp.z, glm::sqrt(psq));
    float bsq = glm::length(temp); // Distance between shoulder and wrist
    float tt = (l2.get() * l2.get() + bsq * bsq - forearm * forearm) / (2 * l2.get() * bsq);
    //LogInfo(tt);
    float q2 = glm::acos(tt);
    //LogInfo(q2)
    float a2 = (q1 + q2 + glm::pi<float>() * 3.0 / 2.0);
    g2.set(glm::degrees(a2));

    tt = (l2.get() * l2.get() + forearm * forearm - bsq * bsq) /
        (2 * l2.get() * forearm);
    //LogInfo(tt);
    float a3 = glm::acos(tt) + glm::pi<float>();
    g3.set(glm::degrees(a3));

    //onInverse = true;

    mat3 ori = glm::rotate(a1, glm::vec3(0, 0, 1)) * glm::rotate(a2 + a3, glm::vec3(0, 1, 0));

    vec3 zf = vec3(resultPos.get()[2]);
    vec3 yf = vec3(resultPos.get()[1]);

    vec3 z3 = vec3(ori[2]);
    vec3 y3 = vec3(ori[1]);

    vec3 y5 = glm::cross(z3, zf);

    g4.set(glm::degrees(angCalc(y3, y5, z3)));

    g5.set(glm::degrees(angCalc(z3, zf, y5)));

    g6.set(glm::degrees(angCalc(y5, yf, zf)));

    this->process();
    onInverse = false;
    
}

float RoboticsProcessor::angCalc(vec3 a, vec3 b, vec3 n) {
    return glm::atan2(
        glm::length(glm::cross(a, b)),
        glm::dot(a, b)
    ) * glm::sign(glm::dot(glm::cross(a, b), n));
}

const ProcessorInfo& RoboticsProcessor::getProcessorInfo() const
{
    return processorInfo_;
}

}  // namespace inviwo