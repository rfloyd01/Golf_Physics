#include "gnuplot.h"

Gnuplot::Gnuplot()
{
    // with -persist option you will see the windows as your program ends
    //gnuplotpipe=_popen("gnuplot -persist","w");
    //without that option you will not see the window
    // because I choose the terminal to output files so I don't want to see the window
    gnuplotpipe = _popen("gnuplot -persist", "w");
    if (!gnuplotpipe)
    {
        cerr << ("Gnuplot not found !");
    }
}
Gnuplot::~Gnuplot() {
    fprintf(gnuplotpipe, "exit\n");
    _pclose(gnuplotpipe);
}
void Gnuplot::operator()(const string& command) {
    fprintf(gnuplotpipe, "%s\n", command.c_str());
    fflush(gnuplotpipe);
    // flush is necessary, nothing gets plotted else
};

void graphBallFlight() //this is being used as a test
{
    //This function takes the data stored in the file 'ball_fligh.dat' and graphs it on a 3d plot
    //also plots a line running straight down the target line
    
    //Define size and shape of the plot
    double plot_length = 350; //how long the plot will be
    double plot_width = 100; //how many yards wide the plot will be
    double plot_height = 50; //the maximum height of the plot in yards
    double plot_scale = 2.5; //the higher this value is the larger the graph will be

    //TODO: not sure if this is the best way to do this but create a separate file to graph target line
    //create a line with only two points and connect them with a straight line
    std::ofstream myFile;
    myFile.open("line.dat");
    myFile << 0 << "    " << 0 << "    " << 0 << '\n';
    myFile << plot_length << "    " << 0 << "    " << 0;
    myFile.close();
    std::string function = "splot 'ball_flight.dat' using 1:2:3 with lines, 'line.dat' using 1:2:3 with lines";

    Gnuplot plot;
    //plot("set terminal wxt size 1000,1000"); //make the gnuplot window a square so that all axes tics have the same physical distance between them
    plot("set xrange [0:" + std::to_string(plot_length) + "]");
    plot("set yrange [" + std::to_string(-plot_width / 2.0) + ":" + std::to_string(plot_width / 2.0) + "]");
    plot("set zrange [0:" + std::to_string(plot_height) + "]");
    plot("set ticslevel 0"); //make sure that axes aren't offset at all
    plot("set view equal_axes"); //make the scale of the x and y axes equal to each other
    plot("set view 81,262," + std::to_string(plot_scale) + "," + std::to_string(plot_height / plot_length)); //set the scale of the z-axis to a value of total_z_height / total_x_length so the z-axis looks good on the graph
    plot(function);
}
