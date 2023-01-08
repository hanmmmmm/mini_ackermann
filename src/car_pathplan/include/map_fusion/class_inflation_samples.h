#ifndef CLASS_INFLATION_SAMPLES
#define CLASS_INFLATION_SAMPLES

#include <vector>
#include <iostream>

class ClassInflationSamples
{
private:
    int8_t obs_value;
    int8_t clr_value;
    int8_t obs_value1;
    int8_t obs_value2;
    int8_t obs_value3;
public:
    ClassInflationSamples();
    ~ClassInflationSamples();

    void get_sample_by_radius(std::vector< std::vector<int8_t> >& inflate_sample_target, const int radius);
};

ClassInflationSamples::ClassInflationSamples()
{
    obs_value = 100;
    clr_value = 0; 
    obs_value1 = 90;
    obs_value2 = 80;
    obs_value3 = 70;
}

ClassInflationSamples::~ClassInflationSamples()
{
}

void ClassInflationSamples::get_sample_by_radius(std::vector< std::vector<int8_t> >& inflate_sample_target, const int radius){
    if( radius == 1){
        inflate_sample_target.push_back( {clr_value,obs_value1,clr_value} );
        inflate_sample_target.push_back( {obs_value1,obs_value,obs_value1} );
        inflate_sample_target.push_back( {clr_value,obs_value1,clr_value} );
    }
    else if( radius == 2){
        // inflate_sample_target.push_back( {0,obs_value,obs_value,obs_value,0} );
        // inflate_sample_target.push_back( {obs_value,obs_value,obs_value,obs_value,obs_value} );
        // inflate_sample_target.push_back( {obs_value,obs_value,obs_value,obs_value,obs_value} );
        // inflate_sample_target.push_back( {obs_value,obs_value,obs_value,obs_value,obs_value} );
        // inflate_sample_target.push_back( {0,obs_value,obs_value,obs_value,0} );
        inflate_sample_target.push_back( {clr_value ,clr_value ,obs_value2,clr_value ,clr_value} );
        inflate_sample_target.push_back( {clr_value ,obs_value1,obs_value1,obs_value1,clr_value} );
        inflate_sample_target.push_back( {obs_value2,obs_value1,obs_value ,obs_value1,obs_value2} );
        inflate_sample_target.push_back( {clr_value ,obs_value1,obs_value1,obs_value1,clr_value} );
        inflate_sample_target.push_back( {clr_value ,clr_value ,obs_value2,clr_value ,clr_value} );
    }
    else if( radius >= 3){

        inflate_sample_target.push_back( {clr_value,clr_value,obs_value3,obs_value3,obs_value3,clr_value,clr_value} );
        inflate_sample_target.push_back( {clr_value,obs_value2,obs_value2,obs_value2,obs_value2,obs_value2,clr_value} );
        inflate_sample_target.push_back( {obs_value3,obs_value2,obs_value1,obs_value1,obs_value1,obs_value2,obs_value3} );
        inflate_sample_target.push_back( {obs_value3,obs_value2,obs_value1,obs_value,obs_value1,obs_value2,obs_value3} );
        inflate_sample_target.push_back( {obs_value3,obs_value2,obs_value1,obs_value1,obs_value1,obs_value2,obs_value3} );
        inflate_sample_target.push_back( {clr_value,obs_value2,obs_value2,obs_value2,obs_value2,obs_value2,clr_value} );
        inflate_sample_target.push_back( {clr_value,clr_value,obs_value3,obs_value3,obs_value3,clr_value,clr_value} );
    }
}







#endif