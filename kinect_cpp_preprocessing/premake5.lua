workspace "kinect_cpp_preprocessing"
	architecture "x64"

	configurations
	{
		"Debug",
		"Release"
	}

outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

project "kinect_cpp_preprocessing"
	location "kinect_cpp_preprocessing"
	language "C++"
	kind "ConsoleApp"

	links 
	{
		"opencv_world452d",
		"Kinect20"
	}

	libdirs 
	{
		"vendor/opencv/build/x64/vc15/lib",
		"vendor/kinect/v2.0_1409/Lib/x64"
	}
	
	targetdir("bin/" .. outputdir .. "/%{prj.name}")
	objdir("bin-int/" .. outputdir .. "/%{prj.name}")

	files
	{
		"%{prj.name}/src/**.h",
		"%{prj.name}/src/**.cpp"
	}

	includedirs
	{
		"vendor/kinect/v2.0_1409/inc",
		"vendor/opencv/build/include"
	}

	filter "system:windows"
		cppdialect "C++17"
		systemversion "latest"

		defines "K_PLATFORM_WINDOWS"

		--postbuildcommands
		--{
		--	("call postBuildCommands.bat")
		--}

	filter "configurations:Debug"
		defines "K_DEBUG"
		symbols "On"

	filter "configurations:Release"
		defines "K_RELEASE"
		optimize "On"

