#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <julia.h>

// Define the type of the function calculate_control
typedef double (*calculate_control_t)(double r, double y, double uff);

int main() {
    jl_init();

    const char *lib_path = "/home/fredrikb/.julia/dev/DiscretePIDs/examples/juliac/juliac_pid.so";

    // Load the shared library
    printf("Loading juliac_pid.so\n");
    void *lib_handle = dlopen(lib_path, RTLD_LAZY);
    if (!lib_handle) {
        fprintf(stderr, "Error: Unable to load library %s\n", dlerror());
        exit(EXIT_FAILURE);
    }
    printf("Loaded juliac_pid.so\n");

    // Locate the calculate_control function
    printf("Finding calculate_control!\n");
    calculate_control_t calculate_control = (calculate_control_t)dlsym(lib_handle, "calculate_control!");
    char *error = dlerror();
    if (error != NULL) {
        fprintf(stderr, "Error: Unable to find symbol calculate_control!: %s\n", error);
        dlclose(lib_handle);
        exit(EXIT_FAILURE);
    }
    printf("Found calculate_control!\n");

    // Call the function
    double r = 0.0, y = 0.0, uff = 0.0;
    double result = calculate_control(r, y, uff);
    printf("calculate_control! returned: %f\n", result);

    // Close the library
    dlclose(lib_handle);

    jl_atexit_hook(0);
    return 0;
}


// export LD_LIBRARY_PATH=/home/fredrikb/repos/julia/usr/lib:$LD_LIBRARY_PATH
// gcc -o pid_program test_juliac_pid.c -I /home/fredrikb/repos/julia/usr/include/julia -L/home/fredrikb/repos/julia/usr/lib -ljulia -ldl
