#include "src/Jerk_control.h"
#include <stdio.h>
#include <assert.h>

const static double dt = 0.005;
const static VAJ_set vaj_settings = {70.0, 100.0, 140.0};
const static char *colors[] = {"blue", "red", "green", "black", "purple", "orange"};
static FILE *of;

static bool my_isClose(double a, double b) {
    return fabs(a - b) < 0.001;
}
static void my_test(double distance) {
    static int n_graph = 0;
    n_graph++;
    fprintf(of, "<path stroke=\"%s\" d=\"M0 0", colors[n_graph % (sizeof(colors) / sizeof(colors[1]))]);
    double result;
    jerkType array[8];
    bool finished = false, t4_ed;

    result = 0.0;
    VAJ_process(distance, vaj_settings, array);
    for(unsigned i = 0;!finished;i++) {
        const double speed = velocity_process(dt, i, array, &finished, &t4_ed),
        svgY = (70.0 - speed) * 2;
        fprintf(of, "M %d %f l1 0 ", i, svgY);
        result += speed;
    }
    result *= dt;
    if (my_isClose(distance, result))
        printf("PASSED: expect: %lf, result: %lf\n", distance, result);
    else 
        printf("FAILED: expect: %lf, result: %lf\n", distance, result);
    fputs("\"/>", of);
}
int main()
{
    of = fopen("vaj_graph.svg", "wb");
    fputs(
        "<?xml version=\"1.0\"?>"
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1100\" height=\"500\">",
        of
    );
    if (!of) {
        perror("fopen");
        return 1;
    }
    double test_distances[] = { 20.0, 50.0, 80.0, 100.0, 150.0, 200.0, 500.0, 1000.0, 2000.0 };
    for (size_t i = 0; i < sizeof(test_distances) / sizeof(test_distances[0]); ++i)
        my_test(test_distances[i]);
    fputs("</svg>", of);
    fclose(of);
    return 0;
}
