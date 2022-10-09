set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
set(BUILD_GTEST ON CACHE BOOL "" FORCE)

include(FetchContent)
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

FetchContent_Declare(
        etl
        GIT_REPOSITORY https://github.com/ETLCPP/etl
        GIT_TAG        20.34.0
)
FetchContent_MakeAvailable(etl)

enable_testing()

add_executable(
        native_test
        #test_desktop/main.cpp
        test_desktop/AccelerationRampTest.cpp
        test_desktop/DriverTest.cpp
        test_desktop/StepperTest.cpp
)

target_compile_definitions(native_test PUBLIC F_CPU=16000000)

target_link_libraries(
        native_test
        GTest::gmock_main
        etl
)

include(GoogleTest)
gtest_discover_tests(native_test)