function(get_git_hash _git_hash)
  execute_process(
    COMMAND git log -1 --pretty=format:%h
    OUTPUT_VARIABLE ${_git_hash}
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  set(${_git_hash} ${${_git_hash}} PARENT_SCOPE)
endfunction()

function(get_git_branch _git_branch)
  execute_process(
    COMMAND git symbolic-ref --short -q HEAD
    OUTPUT_VARIABLE ${_git_branch}
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  set(${_git_branch} ${${_git_branch}} PARENT_SCOPE)
endfunction()

function(get_git_tag _git_tag)
  execute_process(
    COMMAND git describe --abbrev=6 --always --tags
    OUTPUT_VARIABLE ${_git_tag}
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  set(${_git_tag} ${${_git_tag}} PARENT_SCOPE)
endfunction()

function(generic_pkg_version version_file_path)
  if(NOT EXISTS ${version_file_path})
    configure_file(
      "${GENERIC_DEVELOPMENT_ENVIRONMENT_ROOT}/include/PkgVersion.h.in"
      "${version_file_path}"
    )
  endif()
endfunction()

set(GIT_HASH "")
set(GIT_BRANCH "")
set(GIT_TAG "")
string(TIMESTAMP COMPILE_TIME "%Y/%m/%d %H:%M:%S")
set(BUILD_TIME ${COMPILE_TIME})
get_git_hash(GIT_HASH)
get_git_branch(GIT_BRANCH)
get_git_tag(GIT_TAG)

