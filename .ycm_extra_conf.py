# This file is NOT licensed under the GPLv3, which is the license for the rest
# of YouCompleteMe.
#
# Here's the license text for this file:
#
# This is free and unencumbered software released into the public domain.
#
# Anyone is free to copy, modify, publish, use, compile, sell, or
# distribute this software, either in source code form or as a compiled
# binary, for any purpose, commercial or non-commercial, and by any
# means.
#
# In jurisdictions that recognize copyright laws, the author or authors
# of this software dedicate any and all copyright interest in the
# software to the public domain. We make this dedication for the benefit
# of the public at large and to the detriment of our heirs and
# successors. We intend this dedication to be an overt act of
# relinquishment in perpetuity of all present and future rights to this
# software under copyright law.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
# OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# For more information, please refer to <http://unlicense.org/>

import os
import ycm_core

def GetRosIncludePaths():
    """Return a list of potential include directories

    The directories are looked for in $ROS_WORKSPACE.
    """
    try:
        from rospkg import RosPack
    except ImportError:
        return []
    rospack = RosPack()
    includes = []

    for p in rospack.list():
        if os.path.exists(rospack.get_path(p) + '/include'):
            includes.append(rospack.get_path(p) + '/include')

    for distribution in os.listdir('/opt/ros'):
        dis_include_path = '/opt/ros/' + distribution + '/include'
        includes.append(dis_include_path)

    ws_include_path = os.path.expandvars('$CATKIN_ROS_WS') + '/install/include'
    includes.append(ws_include_path)

    dev_include_path = os.path.expandvars('$CATKIN_ROS_WS') + '/devel/include'
    includes.append(dev_include_path)

    return includes


def GetRosIncludeFlags():
    includes = GetRosIncludePaths()
    flags = []
    for include in includes:
        flags.append('-isystem')
        flags.append(include)
    return flags

# These are the compilation flags that will be used in case there's no
# compilation database set (by default, one is not set).
# CHANGE THIS LIST OF FLAGS. YES, THIS IS THE DROID YOU HAVE BEEN LOOKING FOR.
default_flags = [
'-Wall',
'-Wextra',
# '-Werror',
'-Wno-long-long',
'-Wno-variadic-macros',
'-fexceptions',
'-DNDEBUG',
# You 100% do NOT need -DUSE_CLANG_COMPLETER in your flags; only the YCM
# source code needs it.
'-DUSE_CLANG_COMPLETER',
# THIS IS IMPORTANT! Without a "-std=<something>" flag, clang won't know which
# language to use when compiling headers. So it will guess. Badly. So C++
# headers will be compiled as C headers. You don't want that so ALWAYS specify
# a "-std=<something>".
# For a C project, you would set this to something like 'c99' instead of
# 'c++11'.
# '-std=c++11',
'-std=c++11',
# ...and the same thing goes for the magic -x option which specifies the
# language that the files to be compiled are written in. This is mostly
# relevant for c++ headers.
# For a C project, you would set this to 'c' instead of 'c++'.
'-x',
'c++',

# c/c++ include path
'-isystem',
'/usr/include/c++/4.8',
'-isystem',
'/usr/include/c++/4.8.5',
'-isystem',
'/usr/include/c++/4.9.3',
'-isystem',
'/usr/include/c++/5',
'-isystem',
'/usr/include/c++/6',
'-isystem',
'/usr/include/c++/7',
'-isystem',
'/usr/include/c++/8',
'-isystem',
'/usr/include/c++/9.1.0',
'-isystem',
'/usr/include',
'-isystem',
'/usr/include/x86_64-linux-gnu',
'-isystem',
'/usr/local/include',
'-isystem',
'/home/gyc/local/include/gazebo-11',
'-isystem',
'/opt/ros/melodic/include',

'-isystem',
'/usr/include/ignition/math2',
'-isystem',
'/usr/include/pcl-1.7',
'-isystem',
'/usr/include/eigen3',
]

"""
flags = [
    '-isystem',
    '/home/gyc/catkin_sweeper/src/along_the_wall/include',
] + default_flags + GetRosIncludeFlags()
"""
flags = default_flags + GetRosIncludeFlags()

def GetCompilationDatabaseFolder(filename):
    """Return the directory potentially containing compilation_commands.json

    Return the absolute path to the folder (NOT the file!) containing the
    compile_commands.json file to use that instead of 'flags'. See here for
    more details: http://clang.llvm.org/docs/JSONCompilationDatabase.html.
    The compilation_commands.json for the given file is returned by getting
    the package the file belongs to.
    """
    try:
        import rospkg
    except ImportError:
        return ''
    pkg_name = rospkg.get_package_name(filename)
    if not pkg_name:
        return ''
    dir = (os.path.expandvars('$ROS_WORKSPACE') +
           os.path.sep +
           'build' +
           os.path.sep +
           pkg_name)

    return dir


def GetDatabase(compilation_database_folder):
    if os.path.exists(compilation_database_folder):
        return ycm_core.CompilationDatabase(compilation_database_folder)
    return None

SOURCE_EXTENSIONS = ['.cpp', '.cxx', '.cc', '.c', '.m', '.mm']


def DirectoryOfThisScript():
    return os.path.dirname(os.path.abspath(__file__))


def MakeRelativePathsInFlagsAbsolute(flags, working_directory):
    if not working_directory:
        return list(flags)
    new_flags = []
    make_next_absolute = False
    path_flags = ['-isystem', '-I', '-iquote', '--sysroot=']
    for flag in flags:
        new_flag = flag

        if make_next_absolute:
            make_next_absolute = False
            if not flag.startswith('/'):
                new_flag = os.path.join(working_directory, flag)

        for path_flag in path_flags:
            if flag == path_flag:
                make_next_absolute = True
                break

            if flag.startswith(path_flag):
                path = flag[len(path_flag):]
                new_flag = path_flag + os.path.join(working_directory, path)
                break

        if new_flag:
            new_flags.append(new_flag)
    return new_flags


def IsHeaderFile(filename):
    extension = os.path.splitext(filename)[1]
    return extension in ['.h', '.hxx', '.hpp', '.hh']


def GetCompilationInfoForHeaderSameDir(headerfile, database):
    """Return compile flags for src file with same base in the same directory
    """
    filename_no_ext = os.path.splitext(headerfile)[0]
    for extension in SOURCE_EXTENSIONS:
        replacement_file = filename_no_ext + extension
        if os.path.exists(replacement_file):
            compilation_info = database.GetCompilationInfoForFile(
                replacement_file)
            if compilation_info.compiler_flags_:
                return compilation_info
    return None


def GetCompilationInfoForHeaderRos(headerfile, database):
    """Return the compile flags for the corresponding src file in ROS

    Return the compile flags for the source file corresponding to the header
    file in the ROS where the header file is.
    """
    try:
        import rospkg
    except ImportError:
        return None
    pkg_name = rospkg.get_package_name(headerfile)
    if not pkg_name:
        return None
    try:
        pkg_path = rospkg.RosPack().get_path(pkg_name)
    except rospkg.ResourceNotFound:
        return None
    filename_no_ext = os.path.splitext(headerfile)[0]
    hdr_basename_no_ext = os.path.basename(filename_no_ext)
    for path, dirs, files in os.walk(pkg_path):
        for src_filename in files:
            src_basename_no_ext = os.path.splitext(src_filename)[0]
            if hdr_basename_no_ext != src_basename_no_ext:
                continue
            for extension in SOURCE_EXTENSIONS:
                if src_filename.endswith(extension):
                    compilation_info = database.GetCompilationInfoForFile(
                        path + os.path.sep + src_filename)
                    if compilation_info.compiler_flags_:
                        return compilation_info
    return None


def GetCompilationInfoForFile(filename, database):
    # The compilation_commands.json file generated by CMake does not have
    # entries for header files. So we do our best by asking the db for flags
    # for a corresponding source file, if any. If one exists, the flags for
    # that file should be good enough.
    # Corresponding source file are looked for in the same package.
    if IsHeaderFile(filename):
        # Look in the same directory.
        compilation_info = GetCompilationInfoForHeaderSameDir(
            filename, database)
        if compilation_info:
            return compilation_info
        # Look in the package.
        compilation_info = GetCompilationInfoForHeaderRos(filename, database)
        if compilation_info:
            return compilation_info
    return database.GetCompilationInfoForFile(filename)


def FlagsForFile(filename):
    database = GetDatabase(GetCompilationDatabaseFolder(filename))
    if database:
        # Bear in mind that compilation_info.compiler_flags_ does NOT return a
        # python list, but a "list-like" StringVec object
        compilation_info = GetCompilationInfoForFile(filename, database)
        if not compilation_info:
            # Return the default flags defined above.
            return {
                'flags': flags,
                'do_cache': True,
            }

        final_flags = MakeRelativePathsInFlagsAbsolute(
            compilation_info.compiler_flags_,
            compilation_info.compiler_working_dir_)
        final_flags += default_flags
    else:
        relative_to = DirectoryOfThisScript()
        final_flags = MakeRelativePathsInFlagsAbsolute(flags, relative_to)

    return {
        'flags': final_flags,
        'do_cache': True
    }

