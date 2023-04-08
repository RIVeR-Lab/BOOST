import os

import shutil


Import("env")


# print(env.Dump())

# print({platformio.workspace_dir})


# Copy sources we want from Manta to local src folder and overwrite
extra_srcs_rel = [
    '../../Manta/MantaExport/HPT5K0Config.cpp',
    '../../Manta/MantaExport/HPT5K0Config.h',
    '../../Manta/MantaExport/HPT5K0DolphinInfo.cpp',
    '../../Manta/MantaExport/HPT5K0DolphinInfo.h',
    '../../Manta/MantaExport/HPT5K0Info.cpp',
    '../../Manta/MantaExport/HPT5K0Info.h',
    '../../Manta/MantaSDK/Orca/src/OrcaHardware/HPT5K0.h',
    '../../Manta/MantaSDK/Orca/src/OrcaHardware/HPT5K0.cpp',
]

extra_srcs_abs = list(map(lambda p: os.path.abspath(p), extra_srcs_rel))
dst_dir = os.path.abspath("src/copied_from_manta_DO_NOT_EDIT")

try:
    # Remove and recreate dest dir
    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir, True)
    os.makedirs(dst_dir)

    # Given a file's path, return the file and one directory up.
    # E.g. given "C:\foo\bar\baz.txt", return "\bar\"
    def get_one_dir_up(path):
        return os.path.split(os.path.split(path)[0])[1]

    # Given the file path '../../../Manta/MantaExport/HPT5K0Config.cpp', copy the file into dst_dir+get_one_dir_up(path)\HPT5K0Config.cpp
    for path in extra_srcs_abs:
        dst = os.path.join(dst_dir, get_one_dir_up(path))
        if not os.path.exists(dst):
            os.makedirs(dst)
        shutil.copy(path, dst)
    print("Files copied successfully.")

# If source and destination are same
except shutil.SameFileError:
    print("Source and destination represents the same file.")
    exit()

# If destination is a directory.
except IsADirectoryError:
    print("Destination is a directory.")
    exit()

# If there is any permission issue
except PermissionError:
    print("Permission denied.")
    exit()
# For other errors
except:
    print("Error occurred while copying file.")
    exit()

# env.BuildSources(

#     os.path.join("$BUILD_DIR", "external", "/Manta/MantaExport"),

#     os.path.join("$PROJECT_DIR", "../../../Manta/MantaExport"),

#     "+<HPT5K0Config.cpp> +<HPT5K0DolphinInfo.cpp> +<HPT5K0Info.cpp>"
# )


# env.BuildSources(

#     os.path.join("$BUILD_DIR", "external", "/Manta/MantaSDK/Orca/src/OrcaHardware"),

#     os.path.join("$PROJECT_DIR", "../../../Manta/MantaSDK/Orca/src/OrcaHardware"),

#     "+<HPT5K0.cpp>"
# )


print("\n\nPre Script DONE\n\n")
