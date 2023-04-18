# From here: https://community.platformio.org/t/how-to-build-got-revision-into-binary-for-version-output/15380

import datetime
import os
import subprocess

Import("env")

VERSION_FILE = 'version.cpp'

result = subprocess.run(['git', 'rev-parse', 'HEAD'], stdout=subprocess.PIPE).stdout.decode('utf-8').strip()

VERSION_CONTENTS = '''
#include "version.h"
std::string Version::getGitCommitSha1() {{
 return "{}";
}}
std::string Version::getBuildTimestamp() {{
 return "{}";
}}
'''.format(result, datetime.datetime.now())

if os.environ.get('PLATFORMIO_SRC_DIR') is not None:
    VERSION_FILE = os.environ.get('PLATFORMIO_SRC_DIR') + os.sep + VERSION_FILE
elif os.path.exists("src"):
    VERSION_FILE = "src" + os.sep + "version_AUTO_GENERATED" + os.sep + VERSION_FILE
else:
    PROJECT_DIR = env.subst("$PROJECT_DIR")
    os.mkdir(PROJECT_DIR + os.sep + "src")
    VERSION_FILE = "src" + os.sep + "version_AUTO_GENERATED" + os.sep + VERSION_FILE

print("Updating {} with version/timestamp...".format(VERSION_FILE))
with open(VERSION_FILE, 'w+') as FILE:
        FILE.write(VERSION_CONTENTS)