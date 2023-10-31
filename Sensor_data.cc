
#include "Sensor_data.h"
#include "gottochblandat.h"
#include <cmath>
#include <iostream>

using namespace sf;
using namespace std;
using namespace rp::standalone::rplidar;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

Sensor_data::Sensor_data()
{
     mu_info = VectorXf::Zero(3);
}

// läser in data från given rplidar returnerar sant om datat hämtades
bool Sensor_data::read_data(RPlidarDriver* lidar)
{

	rplidar_response_measurement_node_hq_t nodes[8192];
	size_t   count = _countof(nodes);
    u_result res{};
	// hämtar data från lidarn till "nodes"
    res = lidar->grabScanDataHq(nodes, count); // kan behövas en timeoutvariabel i denna
    if (IS_OK(res))
    {
        // tömmer gammal data
        rdata.clear();
        thetadata.clear();
        lidar->ascendScanData(nodes, count);
        // lägger in den nya datan i klassvariablerna
        for (int pos = 0; pos < (int)count ; ++pos)
        {
            rdata.push_back(nodes[pos].dist_mm_q2/4.0f);
            thetadata.push_back(-normalize_r((nodes[pos].angle_z_q14 * 90.f / (1 << 14))* 
            3.14159265359 / 180 + 3.14159265359));
        }
    return 1;
    }
    return 0;
}

//konverterar data till landmärken som placeras i mu
// time_per_measure angivet i millisekunder
void Sensor_data::convert_data(float time_per_measure, float time_since_SLAM,
    float velocity, float steer_angle, float lr, float lf, RenderWindow & SLAM_window)
{

//////////////////////////////GRAFIK/////////////////////////////////////
   // SLAM_window.clear();
///////////////////////////////////////////////////////////////////
    // tar bort gamla landmärken
    obs_mu = VectorXf::Zero(mu_info.rows());


    int index{0};
    int index_landmark_begins{0};
    //styrvinkeln räknas om från bågminuter till radianer
    steer_angle = normalize(steer_angle) * 3.14159265359 / 180.0 / 60.0;
    vector<float> xdata;
    vector<float> ydata;
    // bilens masscentrums vinkelhastighet
    float w{velocity / lr * static_cast<float>(sin(atan(tan(steer_angle) *
        lr / (lf + lr))))};

    // beräknar antalet mätpunkter
    float measures{static_cast<float>(rdata.size())};

    // vrider datan efter hur mycket bilen roterat
    for (int i{0}; i <= measures; i++)
    {
        thetadata[i] = normalize_r(thetadata[i] + i / measures * time_per_measure * w / 1000.0);
    }

    // kräng datan efter hur bilen färdats
    for (int i{0}; i <= measures; i++)
    {
        xdata.push_back(rdata[i]*cos(thetadata[i]));
        ydata.push_back(rdata[i]*sin(thetadata[i]));
//////////////////////////////GRAFIK////////////////////////////////////

        sf::CircleShape e{4};
        e.setPosition(((xdata[i]) + velocity *
            sinc_ish(0, w, time_per_measure * i / measures / 1000.0)) - 2,
            ((ydata[i]) + velocity *
            cosc_ish(0, w, time_per_measure * i / measures / 1000.0)) - 2);
        e.setFillColor(sf::Color::Green);
        SLAM_window.draw(e);
///////////////////////////////////////////////////////////////////
    }
    index = 0;
    obs_landm = 0;

    for (float d: rdata)
    {
        // påbörjar identifiering om skllnaden mellan den nuvarande och
        // föregående datapunkten är större än 300 mm
        if ((index > 0) && (abs(d - rdata[index - 1]) > 300 )) // justerbar
        {
            // om nästa eller nästnästa data ligger inom 50 mm igen
            // förutsätts det nuvarande värdet vara felaktigt och det/de
            // skrivs över med medelvärdet av förra och nästa/nästnästa
            if ((index < measures) &&
                (abs(rdata[index + 1] - rdata[index - 1]) < 50 ))
            {
                rdata[index] = (rdata[index + 1] + rdata[index - 1]) / 2;
                xdata[index] = (xdata[index + 1] + xdata[index - 1]) / 2;
                ydata[index] = (ydata[index + 1] + ydata[index - 1]) / 2;
            }
            else
            {
                int datapoints{index - index_landmark_begins - 1};
                
                // fortsätt om objekten som påträffats har mellan 1 och 60 mätpunkter
                if((datapoints > 1) && (datapoints < 61) &&
                (abs(rdata[index - 1] - rdata[index_landmark_begins]) < 100) &&
                (rdata[index - 1] != 0) && (rdata[index_landmark_begins] != 0)) //justerbar
                {
                    // spara undan datan för det upptäckta objektet
                    vector<float> xlandmdata;
                    vector<float> ylandmdata;
                    for (int j = index_landmark_begins; j < index; j++)
                    {
                        xlandmdata.push_back(xdata[j]);
                        ylandmdata.push_back(ydata[j]);
                    }
                    int datapoints{index - index_landmark_begins - 1};
                    float guess_r{};
                    float guess_x{};
                    float guess_y{};

                    // gissar att radien är ca 70% av avståndet mellan de
                    // första och sista uppmätta punkter
                    guess_r = 0.7 * sqrt( //justerbar
                        pow((xlandmdata[0] - xlandmdata[datapoints]), 2) +
                        pow((ylandmdata[0] - ylandmdata[datapoints]), 2));

                    float angle = static_cast<float>(
                        atan2(ylandmdata[datapoints/2], xlandmdata[datapoints/2]));
                
                    // fortsätter om den gissade radien är mellan 5 och 1000 mm
                    if ((guess_r > 5) && (guess_r < 1000))// justerbart
                    {
                    // uppskattar x- & y-positionen som den gisadde radien
                    // från den mittersta punkten i riktningen rakt från rplidarn
                        guess_x = cos(angle) * guess_r + xlandmdata[datapoints/2];
                        guess_y = sin(angle) * guess_r + ylandmdata[datapoints/2];
                        
                        float closest_dist{static_cast<float>(sqrt(
                            pow(xlandmdata[datapoints/2],2)
                            + pow(ylandmdata[datapoints/2],2)))};
                        

                    
                    Landmark result{0,0,0};
                    Landmark guess{guess_x, guess_y, guess_r};

                        if (circle_regression(xlandmdata, ylandmdata, guess, result)) 
                        {
                            // parar ihop observerade med existerande landmärken
                            int landm_index = data_assocation(guess_x, guess_y,
                                mu(0) + velocity * sinc_ish(mu(2) * 3.1415 / 180 / 60, w, time_since_SLAM / 1000.0),
                                mu(1) + velocity * cosc_ish(mu(2) * 3.1415 / 180 / 60, w, time_since_SLAM / 1000.0));
                            
                            float tempx, tempy, tempr, tempobs;
                            bool landm_obs{false};
                            
                        
                            if (((pow(result.ypos,2) + pow(result.xpos,2)) < 2000 * 2000) && // justerbart
                                (result.radius > 40) && (result.radius < 130)  &&
                                (mu_info(landm_index + 1) < 20))
                            {
                                
                                landm_obs = true;
                                tempx  = result.xpos;
                                tempy  = result.ypos;
                                tempr  = (mu_info(landm_index) * 
                                    mu_info(landm_index + 1) + result.radius) / 
                                    (mu_info(landm_index + 1) + 1);
                                tempobs = mu_info(landm_index + 1) + 1;

                    //////////////////////////////GRAFIK/////////////////////////////////////
                                sf::CircleShape e{tempr};
                                e.setPosition(tempx - tempr, tempy - tempr);
                                e.setFillColor(sf::Color::Blue);
                                SLAM_window.draw(e);
                   //////////////////////////////////////////////////////////////////////
                            }
                           /* else if(mu_info(landm_index + 1) < 20) //justerbar
                            {
                                Landmark guess{guess_x, guess_y, 64};
                                if ((circle_regression(xlandmdata, ylandmdata, guess, result)) && // justerbart
                                    ((pow(result.ypos,2) + pow(result.xpos,2)) < 4000 * 4000) &&
                                    ((pow(result.ypos,2) + pow(result.xpos,2)) > closest_dist) && 
                                    (root_mean_square_error(xlandmdata, ylandmdata, result) < 20))
                                {
                                    landm_obs = true;
                                    tempx     = result.xpos;
                                    tempy     = result.ypos;
                                    tempr     = mu_info(landm_index);
                                    tempobs   = mu_info(landm_index + 1);
                    //////////////////////////////GRAFIK/////////////////////////////////////
                                    sf::CircleShape e{64};
                                    e.setPosition(tempx - 64, tempy - 64);
                                    e.setFillColor(sf::Color::Red);
                                    SLAM_window.draw(e);
                    //////////////////////////////////////////////////////////////////////
                                }
                            }*/
                            else if (mu_info(landm_index + 1) >= 20)
                            {
                                if (mu_info(landm_index) < 80.5)
                                    mu_info(landm_index) = 64;
                                else
                                    mu_info(landm_index) = 97;
                                    
                                Landmark guess{guess_x, guess_y, mu_info(landm_index)};
                        
                                if ((circle_regression(xlandmdata, ylandmdata, guess, result)) && // justerbart
                                    (sqrt(pow(result.ypos,2) + pow(result.xpos,2)) > closest_dist))
                                {
                                    landm_obs = true;
                                    tempx     = result.xpos;
                                    tempy     = result.ypos;
                                    tempr     = result.radius;
                                    tempobs   = mu_info(landm_index + 1) + 1;
                                    
                    //////////////////////////////GRAFIK/////////////////////////////////////
                                    sf::CircleShape e{tempr};
                                    e.setPosition(tempx - tempr, tempy - tempr);
                                    e.setFillColor(sf::Color::Green);
                                    SLAM_window.draw(e);
                    //////////////////////////////////////////////////////////////////////
                                }
                            }
                       
                            
                            if (landm_obs)
                            {
                                if (landm_index == 0)
                                {
                                    landm_index = obs_mu.rows();
                                    obs_mu.conservativeResizeLike(VectorXf::Zero(landm_index + 2));
                                    mu_info.conservativeResizeLike(VectorXf::Zero(landm_index + 2));
                                    mu.conservativeResizeLike(VectorXf::Zero(landm_index + 2));
                                    mu(landm_index)     = mu(0) + tempx;
                                    mu(landm_index + 1) = mu(1) + tempy;
                                }
                                obs_landm++;
                                obs_mu(landm_index)      = tempx;
                                obs_mu(landm_index + 1)  = tempy;
                                if(tempobs != 0)
                                {
                                    mu_info(landm_index)     = tempr; 
                                    mu_info(landm_index + 1) = tempobs;
                                }
                            }
                        }
                    }
                }
            index_landmark_begins = index;
            }
        }
    index++;
    }
}
// utför cirkulär regression givet en gissad radie och x- och y-position
// returnerar huruvida regressionen lyckas
bool Sensor_data::circle_regression(vector<float> xdata, vector<float> ydata,
    Landmark guess, Landmark & result)
{
    bool success{};
    int max_iter = 100; //justerbar
    int iter = 0;
    int iter_inner = 0;
    int points{static_cast<int>(xdata.size())};
    float epsilon = 0.05;    //justerbar
    float convlimit = 1000000;  //justerbar
    float factor_up=10.0;
    float factor_down=0.01;
    float lambda = 0.00001;    //justerbar

    float dx,dy,re,ux,uy,uxx,uyy,uxy,ur,XXl,YYl,Nl,dX,dY,dR;
    float A1,A2,A3,B11,B22,B33,B12,B13,B23,C1,C2,C3;

    Landmark cur_landm{0,0,0};
    Landmark new_landm{guess.xpos, guess.ypos, guess.radius};
    float cur_error = root_mean_square_error(xdata, ydata, guess);
    float new_error{100};
    success = true;
    float xavr{};
    float yavr{};
    for (int i = 0; i < points; i++)
    {
        xavr += xdata[i];
        yavr += ydata[i];
    }

    xavr /= points;
    yavr /= points;

new_ieration:
    cur_landm.xpos   = new_landm.xpos;
    cur_landm.ypos   = new_landm.ypos;
    cur_landm.radius = new_landm.radius;
    cur_error        = new_error;
    // avbryt vid för många iterationer
    if (++iter > max_iter)
    {
        goto enough;
    }
    ux = uy = uxx = uyy = uxy = ur = 0;

    // Plockar fram diverse medelvärden
    for (int i=0; i<points; i++)
    {
        dx  =  xdata[i] - cur_landm.xpos;
        dy  =  ydata[i] - cur_landm.ypos;
        re  =  sqrt(dx * dx + dy * dy);
        ux  += dx / re;
        uy  += dy / re;
        uxx += pow(dx / re,2);
        uyy += pow(dy / re,2);
        uxy += dx / re * dy / re;
        ur  += re;
    }
    ux  /= points;
    uy  /= points;
    uxx /= points;
    uyy /= points;
    uxy /= points;
    ur  /= points;

    // Definierar matriserna A,B & C
    A1 = cur_landm.xpos + cur_landm.radius * ux - xavr;
    A2 = cur_landm.ypos + cur_landm.radius * uy - yavr;
    A3 = cur_landm.radius - ur;

retry:

    XXl = uxx + lambda;
    YYl = uyy + lambda;
    Nl = 1 + lambda;

    B11 = sqrt(XXl);
    B12 = uxy / B11;
    B13 = ux  / B11;
    B22 = sqrt(YYl - B12 * B12);
    B23 = (uy - B12 * B13) / B22;
    B33 = sqrt(Nl - B13 * B13 - B23 * B23);

    C1 = A1/B11;
    C2 = (A2 - B12 * C1) / B22;
    C3 = (A3 - B13 * C1 - B23 * C2) / B33;

    dR = C3/B33;
    dY = (C2 - B23 * dR) / B22;
    dX = (C1 - B12 * dY - B13 * dR) / B11;

    // Kollar om vi är inom felet epsilon
    if ((abs(dR) + abs(dX) + abs(dY)) / (1 + cur_landm.radius) < epsilon)
    {
        goto enough;
    }

    // uppdaterar landmärket
    new_landm.xpos = cur_landm.xpos - dX;
    new_landm.ypos = cur_landm.ypos - dY;

    // Kollar om vi är utanför yttre gränsen: convlimit
    // kan justeras för att ta bort dåliga fall
    if (abs(new_landm.xpos) > convlimit || abs(new_landm.ypos) > convlimit)
    {
        success = false;
        goto enough;
    }
    // Om radien är 64 eller 97 uppdateras den inte 
    if ((cur_landm.radius != 64) && (cur_landm.radius != 97))
        new_landm.radius = cur_landm.radius - dR;

    /*
    // eventuellt fel i radien
    if (New.r <= 0.)
    {
        if (++inner > IterMAX) {code = 2;  goto enough;}
        goto try_again;
    }
    */

//

    new_error = root_mean_square_error(xdata, ydata, new_landm);
    // kolla om resultatet förbättrats
    if (new_error < cur_error)
    {
        lambda *= factor_down;
        goto new_ieration;
    }
    else
    {
        if (++iter_inner > max_iter)
        {
            success = false;
            goto enough;
        }
        lambda *= factor_up;
        goto retry;

    }

enough:

    result.xpos   = new_landm.xpos;
    result.ypos   = new_landm.ypos;
    result.radius = new_landm.radius;
    return success;
}

// beräknar det kvadratiska medelvärdet hos skillnaden mellan givna
// koordinater och en given cirkels periferi
float Sensor_data::root_mean_square_error(vector<float> xdata,
    vector<float> ydata, Landmark circle)
{
    float sum{0};
    int points{static_cast<int>(xdata.size())};

    for (int i=0; i<points; i++)
    {
        sum += pow((sqrt(pow((xdata[i] - circle.xpos), 2) +
            pow((ydata[i] - circle.ypos), 2)) - circle.radius),2);
    }
    return sqrt(sum/points);
}


// jämför nya med gammla landmärken och om de är närmre varandra än 350 mm
// antas de vara samma landmärke
int Sensor_data::data_assocation(float lx_pos, float ly_pos, float cx_pos, float cy_pos)
{
    int index{0};
    for (int i{3}; i < mu.rows(); i += 2)
    {
        if (abs(mu(i) - cx_pos - lx_pos) < 350) // justerbar
        {
            if((pow((mu(i) - cx_pos - lx_pos),2) + (pow((mu(i+1) - cy_pos - ly_pos),2))) < (350 * 350)) // justerbar
            {
                index = i;
                break;
            }
        }
    }
    
    return index;
}

vector<int> Sensor_data::get_landmark_info()
{
    vector<int> landminf{};
    
    for(int i{3}; i < mu_info.rows(); i++)
    {
        landminf.push_back(static_cast<int>(mu_info(i)));
    }
    return landminf;
}
