#include "path_planning.h"
#include "path_point.h"
#include "globala_variabler.h"
#include "Polynomial.h"
#include <vector>
#include <cmath>
#include <ctime>
#include <chrono>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <SFML/Graphics.hpp>
#include "Landmark.h"

using namespace std;
using namespace sf;

struct port
{
	int x;
	int y;
	int angle;
};

std::vector<path_point> calculate_path(const std::vector<point> &through_points);
std::vector<point> best_point_positions(std::vector<point> &through_points);
std::vector<point> best_angles(std::vector<point> points);
float calculate_curvature(float dx_dt, float dy_dt, float d2x_dt2, float d2y_dt2);
double calculate_sum_of_squared_curvatures(std::vector<path_point> &points);
double calculate_maximum_curvature(std::vector<path_point> &points);
int number_of_curvatures_greater_than(std::vector<path_point> &points, float curvature_limit);
float calculate_length_of_path(std::vector<path_point> &points);
float calculate_length_of_path(std::vector<point> &points);
point calculate_intersection_between_lines(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4);
float calculate_angle_difference(float first_angle, float second_angle);
bool cylinder_is_in_allowed_sector(int distance, int angle_from_port_to_cylinder, int port_look_angle);
vector<port> find_ports();

// ATT GÖRA: SLÄNG IN HITTANDE AV PORTAR FRÅN CYLINDRAR!!!
std::vector<point> basbana_portar{{0, 0, 0}, {1200, 0, 0}, {2610, 460, 36 * 60}, {3482, 1658, 72 * 60}, 
    {3482, 3141, 108 * 60}, {2610, 4341, 144 * 60}, {1200, 4800, 180 * 60}, {0, 4800, 180 * 60}, 
    {-1200, 4800, 180 * 60}, {-2610, 4341, 216 * 60}, {-3482, 3141, 252 * 60}, {-3482, 1658, 288 * 60}, 
    {-2610, 460, 324 * 60}, {-1200, 0, 0}};
float time_us{};

// Planera väg
void * path_planning(void * threadid)
{
    std::vector<point> port_points{basbana_portar};
        // Prova köra banan baklänges
        std::reverse(port_points.begin(), port_points.end());
        for (point p : port_points)
        {
            p.original_angle += 180 * 60;
            p.original_angle = (p.original_angle + 360 * 60) % (360 * 60);
        }
    while (true)
    {
        mtx.lock();
        bool recalculate_path{glob_recalculate_path};
        bool exit{glob_exit};
        mtx.unlock();
        if (exit)
            break;
        if (recalculate_path)
        {
            std::cout << "Planerar om vägen" << std::endl;
            std::chrono::steady_clock::time_point time_before{std::chrono::steady_clock::now()};
            
            // Hitta portar från cylindrar först
            
            // Planera en väg
            std::vector<path_point> path_points{calculate_path(port_points)};
            std::chrono::steady_clock::time_point time_after{std::chrono::steady_clock::now()};
            time_us = std::chrono::duration_cast<std::chrono::microseconds>(time_after - time_before).count();
            // Uppdatera global väg
            mtx.lock();
            glob_path_points = path_points;
            glob_port_points = port_points;
            glob_recalculate_path = false;
            mtx.unlock();
            //std::cout << "Planerade om vägen" << std::endl;
        }
    }
}

//Diskontinuerligt k
std::vector<path_point> calculate_path(const std::vector<point> &through_points)
{
    std::vector<path_point> path_points{};
    std::vector<point> gate_points{through_points};
    gate_points = best_angles(gate_points);
    gate_points = best_point_positions(gate_points);  // VILL NOG SPARA DE GAMLA positionerNA ÅSSÅ SÅ VI KAN VISA DEM

        // Each pair of points is connected by a curve parametrised by t where t goes from 0 to 1
        for (unsigned int i{0}; i < gate_points.size(); ++i)
        {
            std::vector<path_point> points{};
            std::vector<path_point> best_points{};

            float minimum_total_curvature{INFINITY};
            int k_for_minimum_curvature{0};
            int angle_for_best_curvature{gate_points.at(i).new_angle};

            point current_point{gate_points.at(i)};
            point next_point{gate_points.at((i + 1) % gate_points.size())};

            int angle{current_point.new_angle};
            int next_angle{next_point.new_angle};
            {
                for (int k{100}; k < 10000; k += 100)
                {
                    // f(x) = a3*x^3 + a2*x^2 + a2*x + a0
                    // f(0) = c
                    // f(1) = d
                    // f'(0) = e
                    // f'(1) = g
                    // a0 = c
                    // a1 = e
                    // a2 = 3d - 3c - 2e - g
                    // a3 = -2d + 2c + e + g

                    int c_x = current_point.x_pos;
                    int c_y = current_point.y_pos;
                    int d_x = next_point.x_pos;
                    int d_y = next_point.y_pos;
                    int e_x = k * cos(angle / 60.0 / 180 * 3.1415);
                    int e_y = k * sin(angle / 60.0 / 180 * 3.1415);
                    int g_x = k * cos(next_angle / 60.0 / 180 * 3.1415);
                    int g_y = k * sin(next_angle / 60.0 / 180 * 3.1415);

                    int a0_x = c_x;
                    int a0_y = c_y;
                    int a1_x = e_x;
                    int a1_y = e_y;
                    int a2_x = 3 * d_x - 3 * c_x - 2 * e_x - g_x;
                    int a2_y = 3 * d_y - 3 * c_y - 2 * e_y - g_y;
                    int a3_x = -2 * d_x + 2 * c_x + e_x + g_x;
                    int a3_y = -2 * d_y + 2 * c_y + e_y + g_y;

                    Polynomial x_polynomial{a3_x, a2_x, a1_x, a0_x};
                    Polynomial y_polynomial{a3_y, a2_y, a1_y, a0_y};

                    // Calculate points
                    int num_points{100};
                    for (int p{0}; p < num_points; ++p)
                    {
                        float t{p / static_cast<float>(num_points)};
                        int x_pos{x_polynomial.evaluate(t)};
                        int y_pos{y_polynomial.evaluate(t)};
                        float dx_dt{x_polynomial.evaluate_derivative(t)};
                        float d2x_dt2{x_polynomial.evaluate_second_derivative(t)};
                        float dy_dt{y_polynomial.evaluate_derivative(t)};
                        float d2y_dt2{y_polynomial.evaluate_second_derivative(t)};
                        float curvature{calculate_curvature(dx_dt, dy_dt, d2x_dt2, d2y_dt2)};
                        int point_angle{60 * 180 / 3.141592 * atan2(dy_dt, dx_dt)};
                        path_point new_point{x_pos, y_pos, point_angle, dx_dt, dy_dt, d2x_dt2, d2y_dt2, curvature}; //...och lite här (point_angle)
                        points.push_back(new_point);
                    }

                    double sum_square_curvature{calculate_sum_of_squared_curvatures(points)};
                    double max_curvature{calculate_maximum_curvature(points)};
                    double min_radius{1 / max_curvature};
                    double length{calculate_length_of_path(points)};
                    int num_points_with_excessive_curvature{number_of_curvatures_greater_than(points, 1.0 / 800)};
                    if (sum_square_curvature < minimum_total_curvature && length < 5 * hypot(current_point.x_pos - next_point.x_pos, current_point.y_pos - next_point.y_pos) && num_points_with_excessive_curvature < 10 && min_radius > 400)
                    {
                        minimum_total_curvature = sum_square_curvature;
                        k_for_minimum_curvature = k;
                        angle_for_best_curvature = angle;
                        best_points = points;
                    }
                    // AVLUSNINGSINFO
                    //std::cout << "k:\t" << k << "\twinkel (grader):\t" << (angle / 60.0) << "\t" << sum_square_curvature << "\t" << min_radius << std::endl;

                    points.clear();
                }
            }
            if (best_points.size() == 0)
            {
                std::cout << i << " Hittade ingen väg" << std::endl;
            }
            else
            {
                std::cout << i << " Hittade väg:\tk: " << k_for_minimum_curvature << "\twinkel: (degar) " << (angle_for_best_curvature / 60.0) << std::endl;
            }
            // Spara valt k och vinkel i portpunkten
            gate_points.at((i + 1) % gate_points.size()).k = k_for_minimum_curvature;
            path_points.insert(path_points.end(), best_points.begin(), best_points.end());
        }
    return path_points;
}

// Anropa denna efter best_angles() !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
std::vector<point> best_point_positions(std::vector<point> &through_points)
{
    // Betrakta tre på varandra följande punkter
    // Den mitterstra punkten flyttas i sidled ortogonalt mot dess vinkel max +-200mm
    // för att hamna så nära den räta linjen mellan de andra två punkterna som möjligt
    std::vector<point> new_points{through_points};

    for (int i{0} ; i < through_points.size() ; i++)
    {
        point prev_point{through_points.at((i - 1 + through_points.size()) % through_points.size())};
        point current_point{through_points.at(i)};
        point next_point{through_points.at((i + 1) % through_points.size())};

        int x1{prev_point.x_pos};
        int y1{prev_point.y_pos};
        int x2{next_point.x_pos};
        int y2{next_point.y_pos};
        int x3{current_point.x_pos};
        int y3{current_point.y_pos};
        int x4{x2 + cos(current_point.original_angle / 60.0 / 180 * 3.1415 + 3.1415 / 2)};
        int y4{y2 + sin(current_point.original_angle / 60.0 / 180 * 3.1415 + 3.1415 / 2)};
        point intersection{calculate_intersection_between_lines(x1, y1, x2, y2, x3, y3, x4, y4)};

        float dx{intersection.x_pos - current_point.x_pos};
        float dy{intersection.y_pos - current_point.y_pos};
        float distance{hypot(dx, dy)};
        if (distance < 1)
        {
            // Kan hända vid raksträckor
            // Placera punkten i mitten av porten
            ;
        }
        else
        {
            // Om vinkelskillnaden genom porten är > 20 grader måste vi nog köra igenom mitten av porten
            if (abs(calculate_angle_difference(current_point.new_angle / 60.0, current_point.original_angle / 60.0)) > 20)
            {
                // Åk genom mitten av porten
                ;
            }
            else
            {
                dx /= distance;
                dy /= distance;

                // Begränsa sidförskjutningen baserat på vinkeln som porten körs igenom med
                int distance_limit{100};

                if (distance > distance_limit)
                    distance = distance_limit;

                // Vilket håll?
                dx *= distance;
                dy *= distance;

                new_points.at(i).x_pos += dx;
                new_points.at(i).y_pos += dy;
            }
        }
    }
    return new_points;
}

// Anropa denna innan best_point_positions() !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
std::vector<point> best_angles(std::vector<point> points)
{
    int j{0};
    while(j < 1)
    {
        j++;
        for (int i{0} ; i < points.size() ; i++)
        {
            point prev_point{points.at((i - 1 + points.size()) % points.size())};
            point next_point{points.at((i + 1) % points.size())};
            int dx_prev{points.at(i).x_pos - prev_point.x_pos};
            int dx_next{next_point.x_pos - points.at(i).x_pos};
            int dy_prev{points.at(i).y_pos - prev_point.y_pos};
            int dy_next{next_point.y_pos - points.at(i).y_pos};

            float angle_next{atan2(dy_next, dx_next)};
            float angle_prev{atan2(dy_prev, dx_prev)};

            float x_comp{cos(angle_prev) + cos(angle_next)};
            float y_comp{sin(angle_prev) + sin(angle_next)};
            int new_angle{atan2(y_comp, x_comp) / 3.1415 * 180 * 60};
            if (new_angle < 0)
                new_angle += 360 * 60;

            /*if (new_angle < points.at(i).original_angle - 45 * 60)
            {
                new_angle = points.at(i).original_angle;
            }
            if (new_angle > points.at(i).original_angle + 45 * 60)
            {
                new_angle = points.at(i).original_angle;
            }*/

            if (calculate_angle_difference(new_angle / 60.0, points.at(i).original_angle / 60.0) < -45)
            {
                new_angle = points.at(i).original_angle - 45 * 60;
            }
            else if (calculate_angle_difference(new_angle / 60.0, points.at(i).original_angle / 60.0) > 45)
            {
                new_angle = points.at(i).original_angle + 45 * 60;
            }

            //new_angle = clamp_angle(new_angle, points.at(i).original_angle / 60.0 - 45, points.at(i).original_angle / 60.0 + 45) * 60;

            std::cout << i << "\t" << points.at(i).original_angle << "\t" << new_angle << std::endl;
            points.at(i).new_angle = new_angle;
        }
    }
    std::cout << "\n\n" << std::endl;
    return points;
}

float calculate_curvature(float dx_dt, float dy_dt, float d2x_dt2, float d2y_dt2)
{
    // https://en.wikipedia.org/wiki/Curvature
    // k = (dx_dt*d2y_dt2 - dy_dt*d2x_dt2) / (dx_dt^2 + dy_dt^2)^(3/2)
    float numerator{dx_dt*d2y_dt2 - dy_dt*d2x_dt2};
    float denominator{dx_dt * dx_dt + dy_dt * dy_dt};
    denominator = pow(denominator, 3.0 / 2.0);
    float k = numerator / denominator;
    return k;
}

double calculate_sum_of_squared_curvatures(std::vector<path_point> &points)
{
    double sum{0};
    for (path_point p : points)
    {
        sum += p.curvature * p.curvature;
    }
    return sum;
}

double calculate_maximum_curvature(std::vector<path_point> &points)
{
    double max{-INFINITY};
    for (path_point p : points)
    {
        if (abs(p.curvature) > max)
        {
            max = abs(p.curvature);
        }
    }
    return max;
}

int number_of_curvatures_greater_than(std::vector<path_point> &points, float curvature_limit)
{
    int count{0};
    for (path_point p : points)
        if (abs(p.curvature) > curvature_limit)
            ++count;
    return count;
}

float calculate_length_of_path(std::vector<path_point> &points)
{
    double length{0};
    for (int i{0}; i < points.size(); ++i)
    {
        path_point prev{points.at((i + 1) % points.size())};
        length += hypot(prev.x_pos - points.at(i).x_pos, prev.y_pos - points.at(i).y_pos);
    }
    return length;
}

float calculate_length_of_path(std::vector<point> &points)
{
    double length{0};
    for (int i{0}; i < points.size(); ++i)
    {
        point prev{points.at((i + 1) % points.size())};
        length += hypot(prev.x_pos - points.at(i).x_pos, prev.y_pos - points.at(i).y_pos);
    }
    return length;
}

// Den första linjen går mellan (x1, y1) och (x2, y2)
// Den andra linjen går mellan (x3, y3) och (x4, y4)
point calculate_intersection_between_lines(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
{
    float uppe{(x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)};
    float nere{(x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)};
    float t{uppe / nere};
    float x{x1 + t * (x2 - x1)};
    float y{y1 + t * (y2 - y1)};
    return point{x, y, 0};
}

float calculate_angle_difference(float first_angle, float second_angle)
  {
        float difference = second_angle - first_angle;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return difference;
 }
 
// ANVÄND glob_landmarks OCH glob_landmark_info ISTÄLLET FÖR ATT TA IN EN VEKTOR AV LANDMÄRKESPEKARE!!!
vector<port> find_ports()
{
	vector<port> ports{};
    
	// Mämta landmärkesinformation och spara som landmärkesvektors
	std::vector<Landmark> landmarks{};
	mtx.lock();
	for (int i{0}; i < glob_landmarks.size(); i += 2)
	{
	    int x{glob_landmarks[i]};
	    int y{glob_landmarks[i + 1]};
	    int r{glob_landmark_info[i]};
	    Landmark landmark{x, y, r}; 
	    landmarks.push_back(landmark);
	}
	mtx.unlock();

	// Bestämmer var nästa port får finnas
	bool if_first_port{true};
	int last_angle{0};
	Vector2i last_port_position{-0, 0};

	// NOTERA ATT DEN FÖRSTA PORTEN INTE SKA HA NÅGRA KRAV PÅ VINKEL ELLER POSITION!!!

	while (landmarks.size() >= 2)
	{
		// Sortera cylindrarna baserat på avstånd till föregående port
		sort(landmarks.begin(), landmarks.end(), [last_port_position](Landmark left, Landmark right)
			{
				return hypot(left.xpos - last_port_position.x, left.ypos - last_port_position.y)
					< hypot(right.xpos - last_port_position.x, right.ypos - last_port_position.y);
			});
		// Utgå från den cylinder som är närmast den senaste porten
		auto it_current_landmark{landmarks.begin()};

		// Kontrollera om denna cylinder är inom det tillåtna området
		int distance_from_last_port{static_cast<int>(hypot((*it_current_landmark).xpos - last_port_position.x, (*it_current_landmark).ypos - last_port_position.y))};
		int angle_from_last_port_to_cylinder{static_cast<int>(atan2((*it_current_landmark).ypos - last_port_position.y, (*it_current_landmark).xpos - last_port_position.x) / 3.1415 * 180 * 60)};
		bool cylinder_is_allowed = cylinder_is_in_allowed_sector(distance_from_last_port, angle_from_last_port_to_cylinder, last_angle);
		// Tillåt alltid om det är första hamnen som konstrueras
		if (if_first_port)
			cylinder_is_allowed = true;
		while (!cylinder_is_allowed)
		{
		    cout << "Cylinder inte tillåten, provar nästa" << endl;
		    it_current_landmark++;
		    if (it_current_landmark == landmarks.end())
		    {
			// Slut på cylindrar. Det fanns ingen inom det tillåtna området!
			// Returnera en tom vektor
			cout << "Blyat!!! Ingen cylinder i det tillåtna området!" << endl;
			return ports;
		    }
		    distance_from_last_port = static_cast<int>(hypot((*it_current_landmark).xpos - last_port_position.x, (*it_current_landmark).ypos - last_port_position.y));
		    angle_from_last_port_to_cylinder = static_cast<int>(atan2((*it_current_landmark).ypos - last_port_position.y, (*it_current_landmark).xpos - last_port_position.x) / 3.1415 * 180 * 60);
		    cylinder_is_allowed = cylinder_is_in_allowed_sector(distance_from_last_port, angle_from_last_port_to_cylinder, last_angle);
		}

		bool port_found{false};
		for (auto it{it_current_landmark + 1}; it != landmarks.end(); it++)
		{
		    //if (it == it_current_landmark)
		    //	continue;

		    cout << "it_current_landmark: " << (*it_current_landmark).xpos << ", " << (*it_current_landmark).ypos << "\t";
		    cout << "it: " << (*it).xpos << ", " << (*it).ypos << endl;

		    // LETA BARA EFTER CYLINDRAR INOM EN VISS VINKEL OCH AVSTÅND FRÅN FÖREGÅENDE PORT!!!

		    // Om avståndet till en annan cylinder är i rätt intervall utgör cylindrarna en port tillsammans.
		    // Mellan 80-90 cm
		    double distance{hypot((*it_current_landmark).xpos - (*it).xpos, (*it_current_landmark).ypos - (*it).ypos)};
		    cout << distance << endl;
		    if (distance >= 800 && distance <= 900)
		    {
			// Lägg till en port bestående av detta landmärke och det landmärke som var på lämpligt avstånd
			Vector2i port_position{((*it_current_landmark).xpos + (*it).xpos) / 2, ((*it_current_landmark).ypos + (*it).ypos) / 2};
			Landmark leftl{0,0,0};
			Landmark rightl{0,0,0};
			int angle{0};
			float landmark_angle{atan2((*it_current_landmark).ypos, (*it_current_landmark).xpos)};
			float it_angle{atan2((*it).ypos, (*it).xpos)};
			if (((landmark_angle > it_angle) && ((landmark_angle > 0 && it_angle > 0) || (landmark_angle < 0 && it_angle < 0))) || ((landmark_angle < it_angle) && (landmark_angle < 0 && it_angle > 0)))
			{
				leftl = *it_current_landmark;
				rightl = *it;
			}
			else
			{
				leftl = *it;
				rightl = *it_current_landmark;
			}
			if (leftl.radius < rightl.radius)
			{
				angle = static_cast<int>(atan2((*it).ypos - (*it_current_landmark).ypos, (*it).xpos - (*it_current_landmark).xpos) / 3.1415 * 180 * 60 - 50 * 60);
			}
			else if (rightl.radius < leftl.radius)
			{
				angle = static_cast<int>(atan2((*it).ypos - (*it_current_landmark).ypos, (*it).xpos - (*it_current_landmark).xpos) / 3.1415 * 180 * 60 - 130 * 60);
			}
			else
			{
				angle = static_cast<int>(atan2((*it).ypos - (*it_current_landmark).ypos, (*it).xpos - (*it_current_landmark).xpos) / 3.1415 * 180 * 60 - 90 * 60);
			}
			port p{port_position.x, port_position.y, angle};
			if_first_port = false;
			ports.push_back(p);
			// Se till att de landmärken som använts inte används igen
			// NOTERA att denna lite haxiga erase()-kod bara fungerar om it pekar
			// på ett senare element i vektorn än it_current_landmark!!!
			landmarks.erase(it);
			it_current_landmark = landmarks.erase(it_current_landmark);
			last_angle = angle;
			last_port_position = port_position;
			port_found = true;
			break;
		    }
		}
		// Gör någonting om ingen port hittades?!?!?!
		if (!port_found)
		{
			cout << "Ingen port!" << endl;
			return ports;
		}
	}
	return ports;
}

bool cylinder_is_in_allowed_sector(int distance, int angle_from_port_to_cylinder, int port_look_angle)
{
	cout << angle_from_port_to_cylinder << " ^^^ " << port_look_angle << endl;

	// ATT GÖRA: TA HÄNSYN TILL CYLINDRARNAS DIAMETRAR OCH DESS PÅVERKAN PÅ VAR CYLINDRARNA FÅR VARA

	// Returnerar true om cylindern ligger på ett ok avstånd och inom +-40 grader från föregående ports vinkel

	// Avståndet till cylindern från föregående pårt måste ligga i intervallet 1.2m-3m (övre gräns lite oklar) Övre gräns är 1.8m
	cout << "Avstånd: " << distance << endl;
	if (distance < 1200 || distance > 1800/*5000*/)
		return false;

	// Vinkeln från föregående port till cylindern måste ligga inom +-40 grader från föregående ports vinkel
	// https://stackoverflow.com/questions/12234574/calculating-if-an-angle-is-between-two-angles
	int angle_difference{(angle_from_port_to_cylinder - port_look_angle + 180 * 60) % (360 * 60) - 180 * 60};
	cout << "winkelskillnad: " << angle_difference << endl;
	if (angle_difference < -40 * 60 || angle_difference > 40 * 60)
		return false;

	return true;
}
