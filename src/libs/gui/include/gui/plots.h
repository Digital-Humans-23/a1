#pragma once

#include <functional>

#include "imgui_widgets/implot.h"

// Base class for plots
class Plot {
public:
    std::string plotName;
    ImVector<ImVec2> data;

    Plot(const std::string &name) : plotName(name) {}
};

/**
 * Real time plots are usually meant to plot a stream of incoming data.
 * The constructor of this class takes in a getter function
 */
class RealtimePlot : public Plot {
public:
    RealtimePlot(const std::string &name, std::function<float(float)> getter,
                 unsigned size = 1000)
        : Plot(name), dataGetter(getter), maxSize(size) {
        data.reserve(maxSize);
    }

    // Max buffer size
    unsigned maxSize;

private:
    // This function is used to retrieve data automatically from the source
    std::function<float(float)> dataGetter;
};

struct PlotInfo {
    std::string plotName;

    std::vector<float> xData;
    std::vector<float> yData;

    PlotInfo() {}

    PlotInfo(const std::string &name) : plotName(name) {}

    PlotInfo(const std::string &plotName_, const std::vector<float> &xData_,
             const std::vector<float> &yData_, bool adjustLimits = false)
        : plotName(plotName_), xData(xData_), yData(yData_) {
        if (xData.size() != yData.size()) {
            throw std::runtime_error(
                "PlotInfo: x and y values count mismatch!");
        }
    }
};

struct PlotContext {
    static ImVec2 getLimits(const std::vector<float> &values) {
        assert(values.size() > 0);

        ImVec2 limits;
        auto minIter = std::min_element(values.begin(), values.end());
        limits.x = *minIter;
        auto maxIter = std::max_element(values.begin(), values.end());
        limits.y = *maxIter;

        return limits;
    }

    // Plot name
    std::string plotContextName{""};

    // Labels
    std::string xLabel{""};
    std::string yLabel{""};

    // Plot limits
    ImVec2 xLimits{0.0, 20.0};
    ImVec2 yLimits{0.0, 10.0};

    bool isActive{false};

    // Plots
    std::map<std::string, PlotInfo> plotsTable;

    // Default constructor
    PlotContext() {}

    // Constructor with name initialization
    PlotContext(const std::string &plotContextName_,
                const std::string &xLabel_ = "",
                const std::string &yLabel_ = "")
        : plotContextName(plotContextName_), xLabel(xLabel_), yLabel(yLabel_) {}

    // Get x limits of plot context
    ImVec2 getLimitsX() { return xLimits; }

    ImVec2 getLimitsY() { return yLimits; }

    void addPlot(const PlotInfo &plot, bool updateLimits = false) {
        // Update the limits first
        if (updateLimits) {
            if (plot.xData.size() > 0) {
                ImVec2 otherLimits = getLimits(plot.xData);
                if (plotsTable.size() == 0) {
                    xLimits = otherLimits;
                } else {
                    if (otherLimits.x < xLimits.x) {
                        xLimits.x = otherLimits.x;
                    }
                    if (otherLimits.y > xLimits.y) {
                        xLimits.y = otherLimits.y;
                    }
                }
            }

            if (plot.yData.size() > 0) {
                ImVec2 otherLimits = getLimits(plot.yData);
                if (plotsTable.size() == 0) {
                    yLimits = otherLimits;
                } else {
                    if (otherLimits.x < yLimits.x) {
                        yLimits.x = otherLimits.x;
                    }
                    if (otherLimits.y > yLimits.y) {
                        yLimits.y = otherLimits.y;
                    }
                }
            }
        }

        // Add plot to the table
        plotsTable.emplace(plot.plotName, plot);
    }

    void reset() {
        isActive = false;
        xLimits = ImVec2(0, 20);
        yLimits = ImVec2(0, 10);
        plotsTable.clear();
    }
};