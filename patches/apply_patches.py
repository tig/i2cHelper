# # https://docs.platformio.org/en/latest/projectconf/advanced_scripting.html?highlight=patch%20platform#override-package-files
# from os.path import join, isfile

# Import("env")

# # access to global build environment
# print(env)

# # access to project build environment (is used source files in "src" folder)
# #print(projenv)

# #
# # (Optional) Do not run extra script when IDE fetches C/C++ project metadata
# #
# from SCons.Script import COMMAND_LINE_TARGETS

# if "idedata" in COMMAND_LINE_TARGETS:
#     env.Exit(0)


# FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-arduinoavr")
# patchflag_path = join(FRAMEWORK_DIR, ".patching-done")

# # patch file only if we didn't do it before
# if not isfile(join(FRAMEWORK_DIR, ".patching-done")):
#     original_file = join(FRAMEWORK_DIR, "variants", "standard", "Ethernet.h")
#     patched_file = join("patches", "fix-esp32-issue-2704.patch")

#     assert isfile(original_file) and isfile(patched_file)

#     env.Execute("patch %s %s" % (original_file, patched_file))
#     # env.Execute("touch " + patchflag_path)


#     def _touch(path):
#         with open(path, "w") as fp:
#             fp.write("")

#     env.Execute(lambda *args, **kwargs: _touch(patchflag_path))