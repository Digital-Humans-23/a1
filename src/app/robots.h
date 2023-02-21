#include <string.h>

#include <vector>

namespace crl {
namespace app {
namespace locomotion {

/**
 * Robot file, initial state file, feet name...
 */
class RobotModel {
public:
    RobotModel(std::string name,         //
               std::string fpath,        //
               std::string spath,        //
               std::string fl,           //
               std::string hl,           //
               std::string fr,           //
               std::string hr,           //
               double baseDropHeight,    //
               double baseTargetHeight,  //
               double swingFootHeight)
        : name(name),
          filePath(fpath),
          statePath(spath),
          fl(fl),
          hl(hl),
          fr(fr),
          hr(hr),
          baseDropHeight(baseDropHeight),
          baseTargetHeight(baseTargetHeight),
          swingFootHeight(swingFootHeight) {}

    std::string name;
    std::string filePath;
    std::string statePath;
    std::string fl, hl, fr, hr;
    double baseDropHeight;
    double baseTargetHeight;
    double swingFootHeight;
};

/* global variable */
std::vector<RobotModel> robotModels = {
    // simple robot models
    RobotModel("Dogbot",                                    //
               CRL_DATA_FOLDER "/robots/cora/cora_v4.rbs",  //
               "",                                          //
               "tibia_0",                                   //
               "tibia_1",                                   //
               "tibia_2",                                   //
               "tibia_3",                                   //
               0.51,                                        //
               0.42,                                        //
               0.10),

};

}  // namespace locomotion
}  // namespace app
}  // namespace crl
