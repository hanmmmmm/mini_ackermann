#ifndef CLASS_INFLATION_SAMPLES
#define CLASS_INFLATION_SAMPLES

#include <vector>
#include <iostream>

class ClassInflationSamples
{
private:
    
public:
    ClassInflationSamples();
    ~ClassInflationSamples();

    void get_sample_by_radius(std::vector< std::vector<int8_t> >& inflate_sample_target, const int radius);
};

ClassInflationSamples::ClassInflationSamples()
{
}

ClassInflationSamples::~ClassInflationSamples()
{
}

void ClassInflationSamples::get_sample_by_radius(std::vector< std::vector<int8_t> >& inflate_sample_target, const int radius){
    if( radius == 1){
        // std::vector<int8_t> row;
        // row = {0,100,0};
        inflate_sample_target.push_back( {0,100,0} );
        inflate_sample_target.push_back( {100,100,100} );
        inflate_sample_target.push_back( {0,100,0} );
    }
    else if( radius == 2){
        // inflate_sample_target.push_back( {0,100,100,100,0} );
        // inflate_sample_target.push_back( {100,100,100,100,100} );
        // inflate_sample_target.push_back( {100,100,100,100,100} );
        // inflate_sample_target.push_back( {100,100,100,100,100} );
        // inflate_sample_target.push_back( {0,100,100,100,0} );
        inflate_sample_target.push_back( {0,0,100,0,0} );
        inflate_sample_target.push_back( {0,100,100,100,0} );
        inflate_sample_target.push_back( {100,100,100,100,100} );
        inflate_sample_target.push_back( {0,100,100,100,0} );
        inflate_sample_target.push_back( {0,0,100,0,0} );
    }
    else if( radius >= 3){

        inflate_sample_target.push_back( {  0,  0,100,100,100,  0,  0} );
        inflate_sample_target.push_back( {  0,100,100,100,100,100,  0} );
        inflate_sample_target.push_back( {100,100,100,100,100,100,100} );
        inflate_sample_target.push_back( {100,100,100,100,100,100,100} );
        inflate_sample_target.push_back( {100,100,100,100,100,100,100} );
        inflate_sample_target.push_back( {  0,100,100,100,100,100,  0} );
        inflate_sample_target.push_back( {  0,  0,100,100,100,  0,  0} );
    }
}







#endif