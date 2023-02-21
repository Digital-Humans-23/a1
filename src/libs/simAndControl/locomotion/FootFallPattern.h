#pragma once

#include <locomotion/LeggedRobot.h>
#include <utils/trajectory.h>
#include <utils/utils.h>
#include <imgui_widgets/ImGuizmo.h>

using namespace std;

namespace crl {

class SwingPhaseContainer {
public:
    // this is the limb the swing phase is applied to...
    RobotLimb *limb;

    // keep track of when a contact should be broken (e.g. a foot lifts), and
    // when it should be established again - the swing phase is defined to take
    // place between these two discrete events...
    DynamicArray<pair<double, double>> swingPhases;

public:
    SwingPhaseContainer(RobotLimb *l) { limb = l; }

    void addSwingPhase(double start, double end) {
        // the swing phase must end after it begins!
        if (start >= end) {
            assert(false);
            return;
        }
        swingPhases.push_back(pair<double, double>(start, end));
    }

    void clearSwingPhasesBefore(double t) {
        while (swingPhases.size() > 0 && swingPhases.front().second < t)
            swingPhases.erase(swingPhases.begin());
    }
};

class PeriodicGait {
public:
    // this is the duration of the stride, in seconds...
    double strideDuration = 1.0;

    // trajectories that control variations in body motion trajectories, encoded
    // as a function of the stride phase
    Trajectory1D bodyHeightOffset, bodyForwardLeanOffset,
        bodySidewaysLeanOffset;
    Trajectory1D bodyPitchOffset, bodyRollOffset, bodyYawOffset;

    // and the swing period is represented here relative to the overall phase of
    // the gait cycle... e.g. starts at 0.25 and ends at 0.4, where the gait
    // cycle is normalized to go from 0 to 1.
    DynamicArray<SwingPhaseContainer> swingPhases;

public:
    PeriodicGait() { clearAllBodyOffsetTrajectories(); }

    void addSwingPhaseForLimb(RobotLimb *l, double start, double end) {
        // the following must hold for a swing phase
        //	1) it must start before it ends
        //	2) the interval (start, end) must have a non-zero intersection
        // with (0, 1), so either end is greater than 0 (and start can be either
        // positive or negative), or start is less than 1 (and end can be less
        // than or greater than 1) 	3) end-start must be less than or equal
        // to 1 ( if equal to 1 this means the limb is always in swing)
        if (!(start < end)) {
            assert(false);
            return;
        }
        if (!(end > 0 || start < 1)) {
            assert(false);
            return;
        }
        if (!(end - start)) {
            assert(false);
            return;
        }

        // periodic gaits will admit only one swing phase per gait cycle...
        for (auto ls : swingPhases)
            if (ls.limb == l) {
                assert(false);
                return;
            }

        swingPhases.push_back(SwingPhaseContainer(l));
        swingPhases.back().addSwingPhase(start, end);
    }

    void clearAllBodyOffsetTrajectories() {
        bodyHeightOffset.clear();
        bodyHeightOffset.addKnot(0, 0);
        bodyHeightOffset.addKnot(1, 0);

        bodyForwardLeanOffset.clear();
        bodyForwardLeanOffset.addKnot(0, 0);
        bodyForwardLeanOffset.addKnot(1, 0);

        bodySidewaysLeanOffset.clear();
        bodySidewaysLeanOffset.addKnot(0, 0);
        bodySidewaysLeanOffset.addKnot(1, 0);

        bodyPitchOffset.clear();
        bodyPitchOffset.addKnot(0, 0);
        bodyPitchOffset.addKnot(1, 0);

        bodyRollOffset.clear();
        bodyRollOffset.addKnot(0, 0);
        bodyRollOffset.addKnot(1, 0);

        bodyYawOffset.clear();
        bodyYawOffset.addKnot(0, 0);
        bodyYawOffset.addKnot(1, 0);
    }
};

class ContactPhaseInfo {
private:
    // if inSwing is false, it must be in stance mode
    bool swingMode = false;
    // this is the total length of time for the current swing or stance phase...
    double contactPhaseDuration = 0;
    // this is the amount of time left until this contact phase ends and the
    // next begins...
    double timeLeft = 0;

public:
    /**
     * at any moment in time, each limb is either in swing (the period of time
     * during which there is no contact) mode or in stance (the phase during
     * which there should be a contact with the environment) mode
     */
    ContactPhaseInfo(bool swingMode, double contactPhaseDuration,
                     double timeLeft) {
        if (contactPhaseDuration < 0.0001) contactPhaseDuration = 0.0001;
        boundToRange(timeLeft, 0, contactPhaseDuration);
        this->swingMode = swingMode;
        this->contactPhaseDuration = contactPhaseDuration;
        this->timeLeft = timeLeft;
    }

    bool isSwing() { return swingMode; }

    bool isStance() { return !swingMode; }

    /**
     * time left until the next phase starts
     */
    double getTimeLeft() { return timeLeft; }

    double getDuration() { return contactPhaseDuration; }

    /**
     * relative amount of elapsed time in this swing/stance mode. 0 means it is
     * at the very start, 1 means it is at the end
     */
    double getPercentageOfTimeElapsed() {
        return (contactPhaseDuration - timeLeft) / contactPhaseDuration;
    }
};

/**
 * keep track of the desired pattern of planned contact phases for all limbs.
 * Note this class is aka a footfallpattern, if we're talking pure legged
 * locomotion
 */
class ContactSchedule {
public:
    // swing phases stored in absolute time...
    DynamicArray<SwingPhaseContainer> cs;

public:
    void addSwingPhaseForLimb(RobotLimb *l, double start, double end) {
        if (!(start < end)) {
            assert(false);
            return;
        }
        if (!(end > 0 || start < 1)) {
            assert(false);
            return;
        }
        if (!(end - start)) {
            assert(false);
            return;
        }

        int lIndex = 0;

        // periodic gaits will admit only one swing phase per gait cycle...
        for (auto ls : cs) {
            if (ls.limb == l) break;
            lIndex++;
        }

        if (lIndex == (int)cs.size()) cs.push_back(SwingPhaseContainer(l));

        // now, make sure that this new swing phase starts after the previous
        // one has ended...
        if (cs[lIndex].swingPhases.size() > 0) {
            // if the new swing phase starts right when the previous one ends,
            // merge them...
            if (fabs(cs[lIndex].swingPhases.back().second - start) < 0.0001) {
                cs[lIndex].swingPhases.back().second = end;
                return;
            }
            if (cs[lIndex].swingPhases.back().second > start) {
                assert(false);
                return;
            }
        }

        cs[lIndex].addSwingPhase(start, end);
    }

    void addPeriodicGaitToContactSequence(const PeriodicGait &pg,
                                          double startTime) {
        for (auto ls : pg.swingPhases)
            addSwingPhaseForLimb(
                ls.limb,
                startTime + ls.swingPhases.front().first * pg.strideDuration,
                startTime + ls.swingPhases.front().second * pg.strideDuration);
    }

    SwingPhaseContainer *getSwingPhaseContainerForLimb(RobotLimb *l) {
        for (uint i = 0; i < cs.size(); i++)
            if (cs[i].limb == l) return &cs[i];
        return nullptr;
    }

    /**
     * returns a data structure that can be used to determine, based on absolute
     * time t, if the limb is in swing or stance mode, time remaining for the
     * current contact configuration, etc...
     */
    ContactPhaseInfo getContactPhaseInformation(const RobotLimb *l, double t) {
        SwingPhaseContainer *spc = nullptr;
        for (uint i = 0; i < cs.size(); i++)
            if (cs[i].limb == l) {
                spc = &cs[i];
                break;
            }

        // if we have no information for this limb, we assume no one gave it any
        // swing phases, so it must be allways in stance then...
        if (spc == nullptr) return ContactPhaseInfo(false, 100, 50);

        // if this limb has been registered but it has no swing phases after t,
        // then it should be assumed it will remain in stance for the forseable
        // future...
        if (spc->swingPhases.size() == 0)
            return ContactPhaseInfo(false, 100, 50);

        // now, t might be before any of the swing phases are starting...
        if (spc->swingPhases.front().first > t)
            return ContactPhaseInfo(false, spc->swingPhases[0].first - t,
                                    spc->swingPhases[0].first - t);

        // or it could be after the last swing phase ends...
        if (spc->swingPhases.back().second < t)
            return ContactPhaseInfo(false, 100, 50);

        // if none of the above are true, t must fall between swing phases, or
        // within a swing phase...
        for (int i = 0; i < (int)spc->swingPhases.size(); i++) {
            // see if t falls within a swing phase
            if (t >= spc->swingPhases[i].first &&
                t <= spc->swingPhases[i].second)
                return ContactPhaseInfo(
                    true,
                    spc->swingPhases[i].second - spc->swingPhases[i].first,
                    spc->swingPhases[i].second - t);
            // since swing phases are sorted, if t is smaller than the next
            // swing phase, it means it falls within a contact region... note:
            // we know that t does not start after the last swing phase, so the
            // "i+1" here should be safe...
            if (t <= spc->swingPhases[i + 1].first)
                return ContactPhaseInfo(
                    false,
                    spc->swingPhases[i + 1].first - spc->swingPhases[i].second,
                    spc->swingPhases[i + 1].first - t);
        }

        // all right, we should never get here...
        assert(false);
        // but the show must go on...
        return ContactPhaseInfo(false, 100, 50);
    }
};

typedef ContactSchedule FootFallPattern;

class ContactPlanManager {
public:
    DynamicArray<pair<double, double>> strideUpdates;

    // this is the time when the last periodic ffp was added. When a new one
    // will be added, it will go from there...
    double timeStampForLastUpdate = 0;
    // visualization options here...
    double visTimeWindow = 3;
    // this is the overall window width we want for the visualizer...
    double cpmWindowWidth = 1300;
    double labelWindowWidth = 50;
    bool drawLabels = true;

    // this is the schedule of planned swing and stance phases for all the
    // limbs...
    ContactSchedule cs;

public:
    ContactPlanManager() {}

    void appendPeriodicGaitToPlanningHorizon(const PeriodicGait &pg) {
        cs.addPeriodicGaitToContactSequence(pg, timeStampForLastUpdate);
        strideUpdates.push_back(
            pair<double, double>(timeStampForLastUpdate,
                                 timeStampForLastUpdate + pg.strideDuration));
        timeStampForLastUpdate += pg.strideDuration;
    }

    double timeUntilEndOfPlanningHorizon(double t) {
        return timeStampForLastUpdate - t;
    }

    void tidyUp(double t) {
        for (uint i = 0; i < cs.cs.size(); i++)
            cs.cs[i].clearSwingPhasesBefore(t);
        while (strideUpdates.size() > 0 && strideUpdates.front().second < t)
            strideUpdates.erase(strideUpdates.begin());
    }

    ContactPhaseInfo getCPInformationFor(const RobotLimb *limb, double t) {
        return cs.getContactPhaseInformation(limb, t);
    }

    float getWindowCoord(double tVal, double tStart, double tEnd) {
        double p = (tVal - tStart) / (tEnd - tStart);
        return labelWindowWidth + (float)p * (cpmWindowWidth);
    }

    // use Dear ImGUI code to visualize the contact schedule...
    void visualizeContactSchedule(double t, double gridStartTime = -1, double dt_grid = 1 / 30.0, int nGridSteps = 30)
    {
        ImGui::SetNextWindowBgAlpha(1.0);
        ImGui::Begin("Contact Schedule Visualizer");
        ImGuizmo::BeginFrame();

        //		float time_ = t;
        //		ImGui::SliderFloat("CPM time", &time_, -10, 10);
        //		t = time_;

        ImGui::Checkbox("draw labels", &drawLabels);

        //		ImGui::InputDouble("Time window", &visTimeWindow, 0.5);
        //		if (visTimeWindow < 0.5) visTimeWindow = 0.5;

        //		if (ImGui::Button("Clean up CP"))
        //			tidyUp();

        // this is where the window screen starts...
        ImVec2 p = ImGui::GetCursorScreenPos();
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        double timeStart = t - visTimeWindow * 0.25;
        double timeEnd = t + visTimeWindow;

        // this is for every row which will visualize the foot fall pattern for
        // one robot limb...
        float height = ImGui::GetFrameHeight();
        float radius = height * 0.005f;
        ImU32 col_bg = ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
        draw_list->AddRectFilled(
            ImVec2(p.x, p.y),
            ImVec2(p.x + labelWindowWidth + cpmWindowWidth + 20,
                p.y + cs.cs.size() * height),
            col_bg);

        ImU32 col_text_gray =
            ImGui::GetColorU32(ImVec4(0.8f, 0.8f, 1.0f, 1.0f));
        ImU32 col_line = ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.7f, 1.0f));
        ImU32 col_cursor = ImGui::GetColorU32(ImVec4(1.0f, 0.7f, 0.7f, 1.0f));
        ImU32 col_horizon = ImGui::GetColorU32(ImVec4(0.5f, 0.5f, 0.5f, 1.0f));
        ImU32 col_text_dark =
            ImGui::GetColorU32(ImVec4(0.1f, 0.1f, 0.1f, 1.0f));

        // draw the swing phases now...
        float rowOffset = 0;
        for (auto lcs : cs.cs) {
            ContactPhaseInfo cpi = cs.getContactPhaseInformation(lcs.limb, t);
            for (auto sp : lcs.swingPhases) {
                float start = getWindowCoord(sp.first, timeStart, timeEnd);
                float end = getWindowCoord(sp.second, timeStart, timeEnd);
                if (end > labelWindowWidth&&
                    start < labelWindowWidth + cpmWindowWidth) {
                    ImU32 col =
                        ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.7f, 1.0f));
                    if (start < labelWindowWidth - 5)
                        start = labelWindowWidth - 5;
                    if (end > labelWindowWidth + cpmWindowWidth + 5)
                        end = labelWindowWidth + cpmWindowWidth + 5;
                    // only mark as in swing the relevant one...
                    if (cpi.isSwing() && sp.first <= t && sp.second >= t)
                        col = ImGui::GetColorU32(ImVec4(
                            1.0f - 0.3 * cpi.getPercentageOfTimeElapsed(),
                            0.7f * (cpi.getPercentageOfTimeElapsed()),
                            0.7f * (cpi.getPercentageOfTimeElapsed()), 1.0f));
                    draw_list->AddRectFilled(
                        ImVec2(p.x + start, p.y + rowOffset + height * 0.15),
                        ImVec2(p.x + end, p.y + rowOffset + height * 0.85), col,
                        height * radius);
                }
            }
            rowOffset += height;
        }

        // cover up the loose ends of the swing phases...

        draw_list->AddRectFilled(
            ImVec2(p.x, p.y), ImVec2(p.x + labelWindowWidth, p.y + rowOffset),
            col_bg);
        draw_list->AddRectFilled(
            ImVec2(p.x + labelWindowWidth + cpmWindowWidth, p.y),
            ImVec2(p.x + labelWindowWidth + cpmWindowWidth + 20,
                p.y + rowOffset),
            col_bg);

        // draw the grid, the labels of the limbs, and the time cursor...
        rowOffset = 0;
        draw_list->AddLine(
            ImVec2(p.x, p.y + rowOffset),
            ImVec2(p.x + labelWindowWidth + cpmWindowWidth, p.y + rowOffset),
            col_line);
        for (auto lcs : cs.cs) {
            draw_list->AddText(
                ImVec2(p.x + height * 0.5, p.y + rowOffset + height * 0.1),
                col_text_gray, lcs.limb->name.c_str());
            rowOffset += height;
            draw_list->AddLine(ImVec2(p.x, p.y + rowOffset),
                ImVec2(p.x + labelWindowWidth + cpmWindowWidth,
                    p.y + rowOffset),
                col_line);
        }
        draw_list->AddLine(ImVec2(p.x, p.y), ImVec2(p.x, p.y + rowOffset),
            col_line);
        draw_list->AddLine(ImVec2(p.x + labelWindowWidth, p.y),
            ImVec2(p.x + labelWindowWidth, p.y + rowOffset),
            col_line);
        draw_list->AddLine(
            ImVec2(p.x + labelWindowWidth + cpmWindowWidth, p.y),
            ImVec2(p.x + labelWindowWidth + cpmWindowWidth, p.y + rowOffset),
            col_line);

        for (auto s : strideUpdates) {
            float sVal = getWindowCoord(s.first, timeStart, timeEnd);
            if (sVal >= labelWindowWidth &&
                sVal <= labelWindowWidth + cpmWindowWidth)
                draw_list->AddLine(ImVec2(p.x + sVal, p.y - 5),
                    ImVec2(p.x + sVal, p.y + rowOffset + 5),
                    col_horizon);
        }

        // show where the current moment in time is...
        float cursorVal;

        if (gridStartTime > -1) {
            int N = nGridSteps;
            for (int i = 0; i < N + 1; i++) {
                double t_tmp = gridStartTime + i * dt_grid;
                if (t_tmp >= timeStart) {
                    cursorVal = getWindowCoord(t_tmp, timeStart, timeEnd);

                    if (i == 0 || i == N) {
                        draw_list->AddLine(ImVec2(p.x + cursorVal, p.y),
                            ImVec2(p.x + cursorVal, p.y + rowOffset),
                            col_line, 2);
                        draw_list->AddCircleFilled(ImVec2(p.x + cursorVal, p.y - 5),
                            5, col_line);
                        draw_list->AddCircleFilled(
                            ImVec2(p.x + cursorVal, p.y + rowOffset + 5), 5,
                            col_line);
                        char timeTextForMPC[100];
                        sprintf(timeTextForMPC, "t = %2.2lfs", t_tmp);
                        draw_list->AddText(ImVec2(p.x + cursorVal - height / 2.0,
                            p.y + rowOffset + height * 0.5),
                            col_text_gray, timeTextForMPC);
                    }
                    else {
                        draw_list->AddLine(ImVec2(p.x + cursorVal, p.y),
                            ImVec2(p.x + cursorVal, p.y + rowOffset),
                            col_line, 0.5);
                    }
                }

                if (t_tmp <= t && t < t_tmp + dt_grid && i < N) {
                    double cursorVal_tmp = getWindowCoord(t, timeStart, timeEnd);
                    char timeText[100];
                    sprintf(timeText, "mpc t idx: %d", i);
                    draw_list->AddText(
                        ImVec2(p.x + cursorVal_tmp, p.y + rowOffset - 220),
                        col_cursor, timeText);
                }
            }
        }

        cursorVal = getWindowCoord(t, timeStart, timeEnd);
        draw_list->AddLine(ImVec2(p.x + cursorVal, p.y),
            ImVec2(p.x + cursorVal, p.y + rowOffset),
            col_cursor);
        char timeText[100];
        sprintf(timeText, "t = %2.2lfs", t);
        draw_list->AddText(ImVec2(p.x + cursorVal - height / 2.0,
            p.y + rowOffset + height * 0.5),
            col_cursor, timeText);
        draw_list->AddCircleFilled(ImVec2(p.x + cursorVal, p.y - 5), 5,
            col_cursor);
        draw_list->AddCircleFilled(ImVec2(p.x + cursorVal, p.y + rowOffset + 5),
            5, col_cursor);



        // and also show here the planning horizon...
        float horizonVal =
            getWindowCoord(timeStampForLastUpdate, timeStart, timeEnd);
        if (horizonVal <= labelWindowWidth + cpmWindowWidth) {
            draw_list->AddLine(ImVec2(p.x + horizonVal, p.y),
                ImVec2(p.x + horizonVal, p.y + rowOffset),
                col_horizon);
            char timeText[100];
            sprintf(timeText, "t = %2.2lfs", timeStampForLastUpdate);
            draw_list->AddText(ImVec2(p.x + horizonVal - height / 2.0,
                p.y + rowOffset + height * 0.5),
                col_horizon, timeText);
            draw_list->AddCircleFilled(ImVec2(p.x + horizonVal, p.y - 5), 5,
                col_horizon);
            draw_list->AddCircleFilled(
                ImVec2(p.x + horizonVal, p.y + rowOffset + 5), 5, col_horizon);
        }

        // finally draw the labels of the swing/stance phases...
        rowOffset = 0;
        if (drawLabels)
            for (auto lcs : cs.cs) {
                ContactPhaseInfo cpi =
                    cs.getContactPhaseInformation(lcs.limb, t);
                char text[100];
                if (cpi.isSwing()) {
                    sprintf(text, "p:%2.0lf%%",
                        cpi.getPercentageOfTimeElapsed() * 100);
                    float labelPos = getWindowCoord(
                        t + cpi.getTimeLeft() - cpi.getDuration(), timeStart,
                        timeEnd);
                    draw_list->AddText(ImVec2(p.x + labelPos + 15,
                        p.y + rowOffset + height * 0.075),
                        col_text_dark, text);
                }
                else {
                    if (cpi.getTimeLeft() < 10) {
                        sprintf(text, "t-%2.2lfs", cpi.getTimeLeft());
                        float labelPos = getWindowCoord(t + cpi.getTimeLeft(),
                            timeStart, timeEnd);
                        draw_list->AddText(
                            ImVec2(p.x + labelPos + 15,
                                p.y + rowOffset + height * 0.075),
                            col_text_dark, text);
                    }
                    else {
                        sprintf(text, "inStance");
                        float labelPos = getWindowCoord(t, timeStart, timeEnd);
                        draw_list->AddText(
                            ImVec2(p.x + labelPos + 15,
                                p.y + rowOffset + height * 0.075),
                            col_text_gray, text);
                    }
                }
                rowOffset += height;
            }

        ImGui::End();
    }
};

}  // namespace crl
