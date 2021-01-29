workspace "InverseKinematics"
    architecture "x64"
    startproject "InverseKinematics"

    configurations
    {
        -- only debug for this hobby project
        "Debug",
        "Release"
    }

-- variables
    -- cfg - configuration
outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

project "InverseKinematics"
    location "."
    kind "ConsoleApp"
    language "C"
    staticruntime "off"

    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
    objdir ("bin-int/" .. outputdir .. "/%{prj.name}")
    
    files
    {
        "include/**.h",
        "src/**.c"
    }

    includedirs
    {
        "include",
        "vendor/include"
    }

    libdirs
    {
        "vendor/lib"
    }

    filter "system:linux"
        buildoptions { "-std=gnu11" }
        systemversion "latest"
        toolset "gcc"
        links
        {
            "GL",
            "glfw",
            "dl",
            "m"
        }

    -- everything under this filter only applies to windows
    filter "system:windows"
        systemversion "latest"

        defines
        {
            "PLATFORM_WINDOWS",
            "_USE_MATH_DEFINES"
        }

        -- linkoptions { "opengl32.lib glfw3.lib" }

        links
        {
            "opengl32",
            "glfw3"
        }

    filter { "configurations:Debug" }
        symbols "On"
    
    filter { "configurations:Release" }
        optimize "On"