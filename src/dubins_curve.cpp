#include "dubins_curve.hpp"


#define EPSILON (10e-10)//(10e-10)


#define DUBINS_CURVE 0

typedef enum 
{
    L_SEG = 0,
    S_SEG = 1,
    R_SEG = 2
} SegmentType;

/* The segment types for each of the Path types */
const SegmentType DIRDATA[][3] = {
    { L_SEG, S_SEG, L_SEG },
    { L_SEG, S_SEG, R_SEG },
    { R_SEG, S_SEG, L_SEG },
    { R_SEG, S_SEG, R_SEG },
    { R_SEG, L_SEG, R_SEG },
    { L_SEG, R_SEG, L_SEG }
};

typedef struct 
{
    double alpha;
    double beta;
    double d;
    double sa;
    double sb;
    double ca;
    double cb;
    double c_ab;
    double d_sq;
} DubinsIntermediateResults;

int dubins_word(DubinsIntermediateResults* in, DubinsPathType pathType, double out[3]);
int dubins_intermediate_results(DubinsIntermediateResults* in, double q0[3], double q1[3], double rho);

/**
 * Floating point modulus suitable for rings
 *
 * fmod doesn't behave correctly for angular quantities, this function does
 */
double fmodr( double x, double y)
{
    return x - y*floor(x/y);
}

double mod2pi( double theta )
{
    return fmodr( theta, 2 * M_PI );
}

void dubins_test()
{


    //std::cout << "Dubins_Curve" << std::endl;
}

int dubins_shortest_path(DubinsPath* path, double q0[3], double q1[3], double rho)
{
    //std::cout << "Gkiri:: Dubins:: stage-0.1" << std::endl;
    int i, errcode;
    DubinsIntermediateResults in;
    double params[3];
    double cost;
    double best_cost = INFINITY;
    int best_word = -1;
    errcode = dubins_intermediate_results(&in, q0, q1, rho);
    if(errcode != EDUBOK) {
        return errcode;
    }

    //std::cout << "Gkiri:: Dubins:: stage-0.1" << std::endl;
    path->qi[0] = q0[0];
    path->qi[1] = q0[1];
    path->qi[2] = q0[2];
    path->rho = rho;
    //std::cout << "Gkiri:: Dubins:: stage-0" << std::endl;
    for( i = 0; i < 6; i++ ) {
        DubinsPathType pathType = (DubinsPathType)i;
        errcode = dubins_word(&in, pathType, params);
        if(errcode == EDUBOK) {
            cost = params[0] + params[1] + params[2];
            if(cost < best_cost) {
                best_word = i;
                best_cost = cost;
                path->param[0] = params[0];
                path->param[1] = params[1];
                path->param[2] = params[2];
                path->type = pathType;
            }
        }
    }
    //std::cout << "Gkiri:: Dubins:: stage-1" << std::endl;
    if(best_word == -1) {
        return EDUBNOPATH;
    }
    return EDUBOK;
}

int dubins_path(DubinsPath* path, double q0[3], double q1[3], double rho, DubinsPathType pathType)
{
    int errcode;
    DubinsIntermediateResults in;
    errcode = dubins_intermediate_results(&in, q0, q1, rho);
    if(errcode == EDUBOK) {
        double params[3];
        errcode = dubins_word(&in, pathType, params);
        if(errcode == EDUBOK) {
            path->param[0] = params[0];
            path->param[1] = params[1];
            path->param[2] = params[2];
            path->qi[0] = q0[0];
            path->qi[1] = q0[1];
            path->qi[2] = q0[2];
            path->rho = rho;
            path->type = pathType;
        }
    }
    return errcode;
}


double dubins_path_length( DubinsPath* path )
{
    double length = 0.;
    length += path->param[0];
    length += path->param[1];
    length += path->param[2];
    length = length * path->rho;
    return length;
}

double dubins_segment_length( DubinsPath* path, int i )
{
    if( (i < 0) || (i > 2) )
    {
        return INFINITY;
    }
    return path->param[i] * path->rho;
}

double dubins_segment_length_normalized( DubinsPath* path, int i )
{
    if( (i < 0) || (i > 2) )
    {
        return INFINITY;
    }
    return path->param[i];
} 

DubinsPathType dubins_path_type( DubinsPath* path ) 
{
    return path->type;
}

void dubins_segment( double t, double qi[3], double qt[3], SegmentType type)
{
    double st = sin(qi[2]);
    double ct = cos(qi[2]);
    if( type == L_SEG ) {
        qt[0] = +sin(qi[2]+t) - st;
        qt[1] = -cos(qi[2]+t) + ct;
        qt[2] = t;
    }
    else if( type == R_SEG ) {
        qt[0] = -sin(qi[2]-t) + st;
        qt[1] = +cos(qi[2]-t) - ct;
        qt[2] = -t;
    }
    else if( type == S_SEG ) {
        qt[0] = ct * t;
        qt[1] = st * t;
        qt[2] = 0.0;
    }
    qt[0] += qi[0];
    qt[1] += qi[1];
    qt[2] += qi[2];
}

int dubins_path_sample( DubinsPath* path, double t, double q[3] ,double end_point_segments[6])
{
    /* tprime is the normalised variant of the parameter t */
    double tprime = t / path->rho;
    double qi[3]; /* The translated initial configuration */
    double q1[3]; /* end-of segment 1 */
    double q2[3]; /* end-of segment 2 */

    double temp_q1[3]={0,0,0};
    double temp_q2[3]={0,0,0};
    double temp_q3[3]={0,0,0};

    const SegmentType* types = DIRDATA[path->type];
    double p1, p2;

    if( t < 0 || t > dubins_path_length(path) ) {
        return EDUBPARAM;
    }

    /* initial configuration */
    qi[0] = 0;//path->qi[0];
    qi[1] = 0;//path->qi[1];
    qi[2] = path->qi[2];

    /* generate the target configuration */
    p1 = path->param[0];
    p2 = path->param[1];
    
    #if PATH_SAMPLE
    std::cout << "Gkiri:: dubins_path_sample param[0]=" << p1 <<" param[1]=" << p2 <<std::endl;
    #endif

    dubins_segment( p1,      qi,    q1, types[0] );//this is for updating q1
    #if PATH_SAMPLE
    std::cout << "Gkiri:: dubins_path_sample q1[0]=" << q1[0] <<" q1[1]=" << q1[1] <<std::endl;
    #endif

    dubins_segment( p2,      q1,    q2, types[1] );//this is for updating q2
    #if PATH_SAMPLE
    std::cout << "Gkiri:: dubins_path_sample q2[0]=" << q2[0] <<" q2[1]=" << q2[1] <<std::endl;
    #endif

    if( tprime < p1 ) {
        #if PATH_SAMPLE
        std::cout << "Gkiri:: 11111111111111111111111111111111111111111111111111111111111" << std::endl;
        #endif
        dubins_segment( tprime, qi, q, types[0] );
    }
    else if( tprime < (p1+p2) ) {
        #if PATH_SAMPLE
        std::cout << "Gkiri:: 222222222222222222222222222222222222222222222222222222222222" << std::endl;
        #endif
        dubins_segment( tprime-p1, q1, q,  types[1] );
    }
    else {
        #if PATH_SAMPLE
        std::cout << "Gkiri:: 3333333333333333333333333333333333333333333333333333333333333333333333" << std::endl;
        #endif
        dubins_segment( tprime-p1-p2, q2, q,  types[2] );
    }

    #if PATH_SAMPLE
    std::cout << "Gkiri:: dubins_path_sample temp_q1[0]=" << temp_q1[0] <<" temp_q1[1]=" << temp_q1[1] <<std::endl;
    std::cout << "Gkiri:: dubins_path_sample temp_q2[0]=" << temp_q2[0] <<" temp_q2[1]=" << temp_q2[1] <<std::endl;
    std::cout << "Gkiri:: dubins_path_sample temp_q3[0]=" << temp_q3[0] <<" temp_q3[1]=" << temp_q3[1] <<std::endl;
    #endif

    /* scale the target configuration, translate back to the original starting point */
    q[0] = q[0] * path->rho + path->qi[0];
    q[1] = q[1] * path->rho + path->qi[1];
    q[2] = mod2pi(q[2]);

    #if PATH_SAMPLE
    std::cout << "Gkiri:: dubins_path_sample final samples q[0]=" << q[0] <<" q[1]=" << q[1] <<std::endl;
    #endif

    //Gkiri added end points updates for segments
    end_point_segments[0]=q1[0]* path->rho + path->qi[0];
    end_point_segments[1]=q1[1]* path->rho + path->qi[1];
    end_point_segments[2]=mod2pi(q1[2]);
    end_point_segments[3]=q2[0]* path->rho + path->qi[0];//end_point_segments[0];
    end_point_segments[4]=q2[1]* path->rho + path->qi[1];//end_point_segments[1];
    end_point_segments[5]=mod2pi(q2[2]);
    
    #if PATH_SAMPLE
    std::cout << "Gkiri:: dubins_path_sample end_point_segments[0]=" << end_point_segments[0] <<" end_point_segments[1]=" << end_point_segments[1] <<std::endl;
    std::cout << "Gkiri:: dubins_path_sample end_point_segments[3]=" << end_point_segments[3] <<" end_point_segments[4]=" << end_point_segments[4] <<std::endl;
    #endif

    return EDUBOK;
}


int dubins_path_sample_many(DubinsPath* path, double stepSize, 
                            DubinsPathSamplingCallback cb, void* user_data,double end_point_segments[6])
{
    int retcode;
    double q[3];
    //double end_point_segments[6];
    double x = 0.0;
    double length = dubins_path_length(path);
    while( x <  length ) {
        dubins_path_sample( path, x, q ,end_point_segments);
        retcode = cb(q, x, user_data,path,end_point_segments);
        if( retcode != 0 ) {
            return retcode;
        }

        x += stepSize;
    }
    return 0;
}

int dubins_path_endpoint( DubinsPath* path, double q[3] )
{
    double end_point_segments[6];
    return dubins_path_sample( path, dubins_path_length(path) - EPSILON, q,end_point_segments );
}

int dubins_extract_subpath( DubinsPath* path, double t, DubinsPath* newpath )
{
    /* calculate the true parameter */
    double tprime = t / path->rho;

    if((t < 0) || (t > dubins_path_length(path)))
    {
        return EDUBPARAM; 
    }

    /* copy most of the data */
    newpath->qi[0] = path->qi[0];
    newpath->qi[1] = path->qi[1];
    newpath->qi[2] = path->qi[2];
    newpath->rho   = path->rho;
    newpath->type  = path->type;

    /* fix the parameters */
    newpath->param[0] = fmin( path->param[0], tprime );
    newpath->param[1] = fmin( path->param[1], tprime - newpath->param[0]);
    newpath->param[2] = fmin( path->param[2], tprime - newpath->param[0] - newpath->param[1]);
    return 0;
}

int dubins_intermediate_results(DubinsIntermediateResults* in, double q0[3], double q1[3], double rho)
{
    //std::cout << "Gkiri:: intermediate :: stage-0" << std::endl;
    
    double dx, dy, D, d, theta, alpha, beta;
    if( rho <= 0.0 ) {
        return EDUBBADRHO;
    }

    dx = q1[0] - q0[0];
    dy = q1[1] - q0[1];
    D = sqrt( dx * dx + dy * dy );
    d = D / rho;
    theta = 0;
    //std::cout << "Gkiri:: intermediate :: stage-1" << std::endl;
    /* test required to prevent domain errors if dx=0 and dy=0 */
    if(d > 0) {
        theta = mod2pi(atan2( dy, dx ));
    }
    alpha = mod2pi(q0[2] - theta);
    beta  = mod2pi(q1[2] - theta);
    //std::cout << "Gkiri:: intermediate :: stage-2" << std::endl;
    in->alpha = alpha;
    in->beta  = beta;
    in->d     = d;
    in->sa    = sin(alpha);
    in->sb    = sin(beta);
    in->ca    = cos(alpha);
    in->cb    = cos(beta);
    in->c_ab  = cos(alpha - beta);
    in->d_sq  = d * d;
    //std::cout << "Gkiri:: intermediate :: stage-3" << std::endl;
    return EDUBOK;
}

int dubins_LSL(DubinsIntermediateResults* in, double out[3]) 
{
    double tmp0, tmp1, p_sq;
    
    tmp0 = in->d + in->sa - in->sb;
    p_sq = 2 + in->d_sq - (2*in->c_ab) + (2 * in->d * (in->sa - in->sb));

    if(p_sq >= 0) {
        tmp1 = atan2( (in->cb - in->ca), tmp0 );
        out[0] = mod2pi(tmp1 - in->alpha);
        out[1] = sqrt(p_sq);
        out[2] = mod2pi(in->beta - tmp1);
        return EDUBOK;
    }
    return EDUBNOPATH;
}


int dubins_RSR(DubinsIntermediateResults* in, double out[3]) 
{
    double tmp0 = in->d - in->sa + in->sb;
    double p_sq = 2 + in->d_sq - (2 * in->c_ab) + (2 * in->d * (in->sb - in->sa));
    if( p_sq >= 0 ) {
        double tmp1 = atan2( (in->ca - in->cb), tmp0 );
        out[0] = mod2pi(in->alpha - tmp1);
        out[1] = sqrt(p_sq);
        out[2] = mod2pi(tmp1 -in->beta);
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_LSR(DubinsIntermediateResults* in, double out[3]) 
{
    double p_sq = -2 + (in->d_sq) + (2 * in->c_ab) + (2 * in->d * (in->sa + in->sb));
    if( p_sq >= 0 ) {
        double p    = sqrt(p_sq);
        double tmp0 = atan2( (-in->ca - in->cb), (in->d + in->sa + in->sb) ) - atan2(-2.0, p);
        out[0] = mod2pi(tmp0 - in->alpha);
        out[1] = p;
        out[2] = mod2pi(tmp0 - mod2pi(in->beta));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_RSL(DubinsIntermediateResults* in, double out[3]) 
{
    double p_sq = -2 + in->d_sq + (2 * in->c_ab) - (2 * in->d * (in->sa + in->sb));
    if( p_sq >= 0 ) {
        double p    = sqrt(p_sq);
        double tmp0 = atan2( (in->ca + in->cb), (in->d - in->sa - in->sb) ) - atan2(2.0, p);
        out[0] = mod2pi(in->alpha - tmp0);
        out[1] = p;
        out[2] = mod2pi(in->beta - tmp0);
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_RLR(DubinsIntermediateResults* in, double out[3]) 
{
    double tmp0 = (6. - in->d_sq + 2*in->c_ab + 2*in->d*(in->sa - in->sb)) / 8.;
    double phi  = atan2( in->ca - in->cb, in->d - in->sa + in->sb );
    if( fabs(tmp0) <= 1) {
        double p = mod2pi((2*M_PI) - acos(tmp0) );
        double t = mod2pi(in->alpha - phi + mod2pi(p/2.));
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(in->alpha - in->beta - t + mod2pi(p));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_LRL(DubinsIntermediateResults* in, double out[3])
{
    double tmp0 = (6. - in->d_sq + 2*in->c_ab + 2*in->d*(in->sb - in->sa)) / 8.;
    double phi = atan2( in->ca - in->cb, in->d + in->sa - in->sb );
    if( fabs(tmp0) <= 1) {
        double p = mod2pi( 2*M_PI - acos( tmp0) );
        double t = mod2pi(-in->alpha - phi + p/2.);
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(mod2pi(in->beta) - in->alpha -t + mod2pi(p));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_word(DubinsIntermediateResults* in, DubinsPathType pathType, double out[3]) 
{
    int result;
    switch(pathType)
    {
    case LSL:
        result = dubins_LSL(in, out);
        break;
    case RSL:
        result = dubins_RSL(in, out);
        break;
    case LSR:
        result = dubins_LSR(in, out);
        break;
    case RSR:
        result = dubins_RSR(in, out);
        break;
    case LRL:
        result = dubins_LRL(in, out);
        break;
    case RLR:
        result = dubins_RLR(in, out);
        break;
    default:
        result = EDUBNOPATH;
    }
    return result;
}


//Helper functions

int printConfiguration(double q[3], double x, void* user_data ,DubinsPath* dub_path,double end_point_segments[6]) {
    //printf("%f,%f,%f,%f\n", q[0], q[1], q[2], x);

    Path *path=(Path *)user_data;
    double rho=10;
    
    switch(dub_path->type)
    {
    case LSL:

        if((q[0] <= end_point_segments[0] && q[1] <= end_point_segments[1]))
        {
            //std::cout << "Gkiri:: LSL 1st arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],-rho);

        }
        else if((q[0] >= end_point_segments[0] && q[1] >= end_point_segments[1]) && (q[0] <= end_point_segments[3] && q[1] <= end_point_segments[4]))
        {
            //std::cout << "Gkiri:: LSL 2nd arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],0);
        }
        else
        {   
            //std::cout << "Gkiri:: LSL 3nd arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],rho);
        }

        break;

    case RSL:

        if((q[0] <= end_point_segments[0] && q[1] <= end_point_segments[1]))
        {
            //std::cout << "Gkiri:: LSL 1st arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],-rho);

        }
        else if((q[0] >= end_point_segments[0] && q[1] >= end_point_segments[1]) && (q[0] <= end_point_segments[3] && q[1] <= end_point_segments[4]))
        {
            //std::cout << "Gkiri:: LSL 2nd arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],0);
        }
        else
        {   
            //std::cout << "Gkiri:: LSL 3nd arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],rho);
        }

        break;

    case LSR:

        if((q[0] <= end_point_segments[0] && q[1] <= end_point_segments[1]))
        {
            //std::cout << "Gkiri:: LSL 1st arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],-rho);

        }
        else if((q[0] >= end_point_segments[0] && q[1] >= end_point_segments[1]) && (q[0] <= end_point_segments[3] && q[1] <= end_point_segments[4]))
        {
            //std::cout << "Gkiri:: LSL 2nd arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],0);
        }
        else
        {   
            //std::cout << "Gkiri:: LSL 3nd arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],rho);
        }

        break;

    case RSR:

        if((q[0] <= end_point_segments[0] && q[1] <= end_point_segments[1]))
        {
            //std::cout << "Gkiri:: LSL 1st arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],-rho);

        }
        else if((q[0] >= end_point_segments[0] && q[1] >= end_point_segments[1]) && (q[0] <= end_point_segments[3] && q[1] <= end_point_segments[4]))
        {
            //std::cout << "Gkiri:: LSL 2nd arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],0);
        }
        else
        {   
            //std::cout << "Gkiri:: LSL 3nd arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],rho);
        }

        break;

    case LRL:

        if((q[0] <= end_point_segments[0] && q[1] <= end_point_segments[1]))
        {
            //std::cout << "Gkiri:: LSL 1st arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],-rho);

        }
        else if((q[0] >= end_point_segments[0] && q[1] >= end_point_segments[1]) && (q[0] <= end_point_segments[3] && q[1] <= end_point_segments[4]))
        {
            //std::cout << "Gkiri:: LSL 2nd arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],rho);
        }
        else
        {   
            //std::cout << "Gkiri:: LSL 3nd arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],rho);
        }

        break;
        
    case RLR:

        if((q[0] <= end_point_segments[0] && q[1] <= end_point_segments[1]))
        {
            //std::cout << "Gkiri:: LSL 1st arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],-rho);

        }
        else if((q[0] >= end_point_segments[0] && q[1] >= end_point_segments[1]) && (q[0] <= end_point_segments[3] && q[1] <= end_point_segments[4]))
        {
            //std::cout << "Gkiri:: LSL 2nd arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],rho);
        }
        else
        {   
            //std::cout << "Gkiri:: LSL 3nd arc " << std::endl;
            path->points.emplace_back(x, q[0], q[1], q[2],rho);
        }

        break;

    default:
         break;
    } 

    /*
    std::cout << "Gkiri::-------LSL:: q[0]=" << q[0] << "end_point_segments[0]= " << end_point_segments[0] << std::endl;
    std::cout << "Gkiri::-------LSL:: q[1]=" << q[1] << "end_point_segments[1]= " << end_point_segments[1] << std::endl;

    std::cout << "Gkiri::-------LSL:: q[3]=" << q[3] << "end_point_segments[3]= " << end_point_segments[3] << std::endl;
    std::cout << "Gkiri::-------LSL:: q[4]=" << q[4] << "end_point_segments[4]= " << end_point_segments[4] << std::endl;
    */
      return 0;
  }


bool dubins_wrapper_api(Path& path,struct arc_extract three_seg[3],double q0[3],double q1[3],double rho)
  {
    #if DUBINS_CURVE
    std::cout << "Gkiri:: planPath:: Started" << std::endl;
    #endif

    double end_point_segments[6];
 
    #if DUBINS_CURVE
    std::cout << "Gkiri:: planPath:: stage-0" << std::endl;
    #endif

    DubinsPath dub_path;
    dubins_shortest_path(&dub_path,  q0,  q1,  rho);
    
    #if DUBINS_CURVE
    printf("#x,y,theta,t\n");
    #endif   

    //SPAGUETI CODE --> To round to second decimal and fix lenght to meters
    DubinsPath dub_path_algo;
    dub_path_algo=dub_path;

    // for (int i=0;i<3;i++){
    //     dub_path.param[i] = dub_path.param[i]/10;
    //     if (dub_path.param[i] < 0.01){
    //         //dub_path.param[i] = 0;
    //     }
    // }
    int update_points;//to update dubins points
    update_points=1;
    dubins_path_sample_many(&dub_path,  0.01, printConfiguration, &path,end_point_segments);

    /*Extracting segments from dubins curve */
    dubins_segments_extract(&dub_path, end_point_segments,rho,three_seg,q1);

    #if DUBINS_CURVE
    std::cout << "Gkiri:: planPath:: dubPath" <<  "q[0]" <<dub_path.qi[0]  << "q[1]" <<dub_path.qi[1] << "q[2]" <<dub_path.qi[2] << std::endl;
    std::cout << "Gkiri:: planPath:: dubPath params[0]=" << dub_path.param[0]/10.0 <<" params[1]=" << dub_path.param[1]/10.0<< " params[2]=" << dub_path.param[2] <<std::endl;
    std::cout << "Gkiri:: planPath:: end_point_segments end_point_segments[0]=" << end_point_segments[0] <<"end_point_segments[1]=" << end_point_segments[1]<< "end_point_segments[2]=" << end_point_segments[2] <<std::endl;
    std::cout << "Gkiri:: planPath:: end_point_segments end_point_segments[3]=" << end_point_segments[3] <<"end_point_segments[4]=" << end_point_segments[4]<< "end_point_segments[5]=" << end_point_segments[5] <<std::endl;

    std::cout << "Gkiri:: planPath:: End" << std::endl;
    #endif

   /* Test Code for Dubins Segment Extract */

    // std::cout << "Gkiri:: planPath:: end_point_segments three_seg[0].start_point.x=" << three_seg[0].start_point.x <<"three_seg[0].start_point.y" << three_seg[0].start_point.y  << std::endl;
    // std::cout << "Gkiri:: planPath:: end_point_segments three_seg[1].start_point.x=" << three_seg[1].start_point.x <<"three_seg[1].start_point.y" << three_seg[1].start_point.y  << std::endl;
    // std::cout << "Gkiri:: planPath:: end_point_segments three_seg[2].start_point.x=" << three_seg[2].start_point.x <<"three_seg[2].start_point.y" << three_seg[2].start_point.y  << std::endl;
    // std::cout << "Gkiri:: planPath:: end_point_segments three_seg[0].center.x=" << three_seg[0].center.x <<"three_seg[0].center.y" << three_seg[0].center.y  << std::endl;
    // std::cout << "Gkiri:: planPath:: end_point_segments three_seg[1].center.x=" << three_seg[1].center.x <<"three_seg[1].center.y" << three_seg[1].center.y  << std::endl;
    // std::cout << "Gkiri:: planPath:: end_point_segments three_seg[2].center.x=" << three_seg[2].center.x <<"three_seg[2].center.y" << three_seg[2].center.y  << std::endl;


    return true;

  }

Point find_center(Point start,Point end,float radius,float length, int LSR)
{   
    //Check if angle between start and end is bigger than 180 degrees (M_PI rad)
    if((length/radius > M_PI)){ // swap start and end
        Point tmp = start;
        start = end;
        end = tmp;
    }

    //https://math.stackexchange.com/questions/27535/how-to-find-center-of-an-arc-given-start-point-end-point-radius-and-arc-direc
    #if DUBINS_CURVE
    std::cout << "Gkiri:: find_center:: " << std::endl;
    #endif

    int epsilon = 1; // Arc from start to end --> clockwise e=1 (LSR = 0), e=-1 counter-clockwise (LSR = 2)
    
    switch (LSR)
    {
    case 0: // counter-clockwise

        //epsilon = 1;
        break;
    case 1: // Straight line --> Center = 0
        return Point(0,0); 
        break;

    case 2: 
        epsilon = -1; 
        break;
        
    default:
        printf("unkown LSR");
        break;
    }    

    Point mid_point(0,0);
    mid_point.x=(start.x+end.x)/2.0 ;
    mid_point.y=(start.y+end.y)/2.0;
    //std::cout << "mid: " << mid_point.x << ", " << mid_point.y << std::endl;

    float distance;
    distance=sqrt(pow((end.x-start.x),2)+pow((end.y-start.y),2));
    if (distance <= 0){
        return Point(0,0);
    }
    //std::cout << "distance: " << distance << std::endl;

    /* 
    n(u,v) =unit normal in the direction z1 to z0
    n*(-v,u) =unit normal in the direction z0 to z1
    
    */
    Point normal_point(0,0);//
    normal_point.x=(end.x-start.x)/distance ;
    normal_point.y=(end.y-start.y)/distance ;
    //std::cout << "normal point module: " << normal_point.x << ", " << normal_point.y << std::endl;

    Point normal_pointstar(0,0);
    normal_pointstar.x= -normal_point.y;
    normal_pointstar.y= normal_point.x ;
    //std::cout << "normal point vector: " << normal_pointstar.x << ", " << normal_pointstar.y << std::endl;

    /*Let Distance H from midpoint to center */
    float h, h_round, sqrt_content;
    sqrt_content = pow(radius,2) - pow(distance,2)/4;   
    //h_round = std::ceil(sqrt_content * 100.0) / 100.0; //round to the 2nd decimal
    //std::cout << "sqrt_content: " << sqrt_content << std::endl;
    //std::cout << "h_round: " << h_round << std::endl;

    if(sqrt_content < -0.0001) //residual threshold for r2-d2/4
    {
        //std::cout << "Gkiri::Sqrt is NaN or 0 " << h << std::endl;
        sqrt_content =0;
    }else if(sqrt_content < 0.0 && sqrt_content > -0.0001)
    {
        //std::cout << "Gkiri::Sqrt is NaN or 0 " << h << std::endl;
    }
    h=sqrt(sqrt_content);   
    //std::cout << "h: " << h << std::endl;

    //c=𝐦+𝜖 ℎ 𝐧∗ 
    Point center;
    center.x=mid_point.x + epsilon*h*normal_pointstar.x;
    center.y=mid_point.y + epsilon*h*normal_pointstar.y;
    //std::cout << "center: " << center.x << ", " << center.y << std::endl;

    return center;
    
}

float distance_points(Point start,Point end)
{
    float distance;
    distance=sqrt(pow((end.x-start.x),2)+pow((end.y-start.y),2));
    return distance;
}

void dubins_segments_extract(DubinsPath *path, double *end_point_segments,double rho,struct arc_extract *three_seg,double goal[3])
  {
    #if DUBINS_CURVE
    std::cout << "Gkiri:: dubins_segments_extract:: End" << std::endl;
    #endif

    three_seg[0].start_point.x=path->qi[0];
    three_seg[0].start_point.y=path->qi[1];
    three_seg[0].radius=rho;
    three_seg[0].end_point.x=end_point_segments[0];
    three_seg[0].end_point.y=end_point_segments[1];
    three_seg[0].length=path->param[0]*rho;    


    three_seg[1].start_point.x=end_point_segments[0];
    three_seg[1].start_point.y=end_point_segments[1];
    three_seg[1].radius=0;
    three_seg[1].end_point.x=end_point_segments[3];
    three_seg[1].end_point.y=end_point_segments[4];
    three_seg[1].length=path->param[1]*rho;    


    three_seg[2].start_point.x=end_point_segments[3];
    three_seg[2].start_point.y=end_point_segments[4];
    three_seg[2].radius=rho;
    three_seg[2].end_point.x=goal[0];
    three_seg[2].end_point.y=goal[1];
    three_seg[2].length=path->param[2]*rho;    


    switch(path->type)
    {
    case LSL:
        three_seg[0].LSR=L_SEG;
        three_seg[1].LSR=S_SEG;
        three_seg[2].LSR=L_SEG;
        break;
    case RSL:
        three_seg[0].LSR=R_SEG;//one for R
        three_seg[1].LSR=S_SEG;
        three_seg[2].LSR=L_SEG;
        break;
    case LSR:
        three_seg[0].LSR=L_SEG;//zero for L
        three_seg[1].LSR=S_SEG;
        three_seg[2].LSR=R_SEG;
        break;
    case RSR:
        three_seg[0].LSR=R_SEG;//zero for L
        three_seg[1].LSR=S_SEG;
        three_seg[2].LSR=R_SEG;
        break;
    case LRL:
         three_seg[0].LSR=L_SEG;//zero for L
         three_seg[1].LSR=R_SEG;
         three_seg[2].LSR=L_SEG;

         three_seg[1].radius=rho;//R
         break;
    case RLR:
        three_seg[0].LSR=R_SEG;//zero for L
        three_seg[1].LSR=L_SEG;
        three_seg[2].LSR=R_SEG;

        three_seg[1].radius=rho;
        break;
    default:
         break;
    }

    //Calculate center
    three_seg[0].center=find_center(three_seg[0].start_point,three_seg[0].end_point,three_seg[0].radius,three_seg[0].length,three_seg[0].LSR);
    three_seg[1].center=find_center(three_seg[1].start_point,three_seg[1].end_point,three_seg[1].radius,three_seg[1].length,three_seg[1].LSR);
    three_seg[2].center=find_center(three_seg[2].start_point,three_seg[2].end_point,three_seg[2].radius,three_seg[2].length,three_seg[2].LSR);

    //Handling small distance cases
    //float min_dist=0.1;
    //@Alvaro
    float min_dist=0.01;
    float dist;
    for(int k=0;k<3;k++)
    {
        dist=distance_points(three_seg[k].start_point,three_seg[k].end_point);
        if(dist<min_dist){
            three_seg[k].end_point.x=three_seg[k].start_point.x;
            three_seg[k].end_point.y=three_seg[k].start_point.y ;
            three_seg[k].center={0.0,0.0};
            three_seg[k].radius=0.0;

            if(k!=2){//Avoiding over buffer
                three_seg[k+1].start_point.x =three_seg[k].end_point.x;
                three_seg[k+1].start_point.y =three_seg[k].end_point.y; 
            }           

        }

    }



  }