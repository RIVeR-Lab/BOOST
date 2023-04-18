// From here: https://community.platformio.org/t/how-to-build-got-revision-into-binary-for-version-output/15380
#pragma once
#include <string>
class Version {
  public:
    static std::string getGitCommitSha1();
    static std::string getBuildTimestamp();
};