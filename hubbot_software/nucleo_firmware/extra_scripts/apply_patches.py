from os.path import join, isfile

Import("env")

FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-arduino-samd-seeed")
PROJ_DIR = env.get('PROJECT_DIR')

original_files = [
    join(FRAMEWORK_DIR, "cores\\arduino\\SERCOM.cpp"),
    join(FRAMEWORK_DIR, "cores\\arduino\\SERCOM.h"),
    join(FRAMEWORK_DIR, "libraries\\Wire\\Wire.cpp"),
]
patched_files = [
    join(PROJ_DIR, "src", "patches", "packages\\framework-arduino-samd-seeed", "cores\\arduino\\SERCOM.cpp"),
    join(PROJ_DIR, "src", "patches", "packages\\framework-arduino-samd-seeed", "cores\\arduino\\SERCOM.h"),
    join(PROJ_DIR, "src", "patches", "packages\\framework-arduino-samd-seeed", "libraries\\Wire\\Wire.cpp")
]
patch_files = [
    join(PROJ_DIR, "src", "patches", "packages\\framework-arduino-samd-seeed", "cores\\arduino\\SERCOM.cpp.patch_generated"),
    join(PROJ_DIR, "src", "patches", "packages\\framework-arduino-samd-seeed", "cores\\arduino\\SERCOM.h.patch_generated"),
    join(PROJ_DIR, "src", "patches", "packages\\framework-arduino-samd-seeed", "libraries\\Wire\\Wire.cpp.patch_generated")
]
for i in range(0, len(original_files)):
    env.Execute("diff %s %s > %s" % (original_files[i], patched_files[i], patch_files[i]))

assert len(original_files) == len(patch_files) == len(patched_files)

for i in range(0, len(original_files)):
    original_file = original_files[i]
    patched_file = patch_files[i]
    assert isfile(original_file) and isfile(patched_file)

    if env.Execute("patch -l %s %s" % (original_file, patched_file)):
        print("SCRIPT FAILED")
        exit(1)


def _touch(path):
    with open(path, "w") as fp:
        fp.write("")

print("\n\nPATCHIGN DONE\n\n")

# env.Execute(lambda *args, **kwargs: _touch(patchflag_path))