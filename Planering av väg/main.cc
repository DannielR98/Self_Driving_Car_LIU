
/*
    Planering av väg givet ett antal punkter som ska köras igenom
    (portarnas positioner) där varje punkt definieras av en position
    samt en riktning som den ska passeras med

    Gustav 2020-02-26
*/

#include <vector>
#include <cmath>
#include <cmath>
#include <ctime>
#include <chrono>
#include <sstream>
#include <iostream>
#include <SFML/Graphics.hpp>
//#include "spline.h"
#include "Polynomial.h"

struct point;
struct path_point;
int main();
std::vector<point> generate_simple_path(const std::vector<point> &through_points);
std::vector<point> generate_path(const std::vector<point> &through_points);
int calculate_angle(int dx, int dy);
std::vector<point> calculate_extra_points(const std::vector<point> &through_points);
//std::vector<point> generate_another_spline_path(const std::vector<point> &through_points);
std::vector<point> calculate_other_extra_points(const std::vector<point> &through_points);
std::vector<path_point> calculate_path(const std::vector<point> &through_points);
float calculate_curvature(float dx_dt, float dy_dt, float d2x_dt2, float d2y_dt2);
double calculate_sum_of_squared_curvatures(std::vector<path_point> &points);
float calculate_length_of_path(std::vector<path_point> &points);
float calculate_length_of_path(std::vector<point> &points);
double calculate_maximum_curvature(std::vector<path_point> &points);
double abs(double x);
std::vector<point> best_angles(std::vector<point> points);
std::vector<point> best_point_positions(std::vector<point> &through_points);
int number_of_curvatures_greater_than(std::vector<path_point> &points, float curvature_limit);
point calculate_intersection_between_lines(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4);
float clamp_angle(float ang, float min, float max);
float calculate_angle_difference(float first_angle, float second_angle);
path_point closest_point_to_car(std::vector<path_point> points);

struct point
{
    int x_pos;
    int y_pos;
    int original_angle;  // Vinkel som punkten ska passeras med i bågminuter (grader / 60)
                        // Vinkeln 0 är rakt åt höger längs x-Axeln och ökar moturs
    int new_angle;
    int k;      // k
};

// Same as a point except that this struct also saves the derivatives
// to enable calculation of curvature
struct path_point
{
    int x_pos;
    int y_pos;
    float dx_dt;
    float d2x_dt2;
    float dy_dt;
    float d2y_dt2;
    float curvature;
};

const int point_selection_radius{300};
int closest_point{-1}; // Index of the closest point
float distance_to_closest_point{0};

int selected_point{-1};
int dx_to_selected_point{0};    // Distance, not derivative
int dy_to_selected_point{0};

int rotation_speed{25};

int closest_path_point{-1};
float distance_to_closest_path_point{0};
int path_point_selection_radius{50};

bool recalculate_path{true};
float time_us{};
std::vector<path_point> path_points{};

int car_x_pos{3200};
int car_y_pos{3000};
int car_angle{-90*60};
int last_point{400};

const int width{1000};
const int height{1000};

int main()
{
    int frames{0};

    // Basbananen
    std::vector<point> points{{0, 0, 0}, {1200, 0, 0}, {2610, 460, 36 * 60}, {3482, 1658, 72 * 60}, {3482, 3141, 108 * 60}, {2610, 4341, 144 * 60}, {1200, 4800, 180 * 60}, {0, 4800, 180 * 60}, {-1200, 4800, 180 * 60},
    {-2610, 4341, 216 * 60}, {-3482, 3141, 252 * 60}, {-3482, 1658, 288 * 60}, {-2610, 460, 324 * 60},
    {-1200, 0, 0}};

    /*std::vector<point> points {{0,0,0}, {2000,0,25*60}, {3500,1000,65*60},
    {4500,2500,90*60}, {4500,4000,60*115}, {3500,5500,60*150}, {2000,6500,60*180},
    {0,6500,60*210}, {-1500,5500,60*240}, {-2500,4000,60*270}, {-2500,2000,60*295},
     {-1500,500,60*330}};*/
    // Hårdkodade punkter än så länge
    /*std::vector<point> points{{0, 0, 0}, {3000, 0, 45*60},
        {5000, 2000, 90*60}, {5000, 4000, 135*60}, {3000, 6000, 180*60},
        {0, 6000, 225*60}, {-2000, 4000, 270*60}, {-2000, 2000, 315*60}};*/

    // Svår bana
    /*std::vector<point> points{{0, 0, 0}, {1500, 0, 0}, {3500, 500, 90*60},
        {3500, 1500, 30*60}, {5000, 2500, 120*60}, {4000, 4000, 180*60},
        {-1000, 4500, 240*60}, {-2000, 1000, 340*60}};*/

    // Skapa en väg
    sf::Clock game_clock;
    sf::RenderWindow window(sf::VideoMode(width, height), "Points"/*,
        sf::Style::Fullscreen*/);
    sf::Time frame_delay_time {sf::milliseconds(17)};

    sf::Font font{};
    font.loadFromFile("Bebas-Regular.ttf");

    int frame_counter{0};
    while (window.isOpen())
    {
        game_clock.restart();

        sf::View view{};
        view.zoom(10);
        sf::View old_view{};
        old_view = window.getView();
        window.setView(view);

        // Handle events
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
            else if (event.type == sf::Event::MouseButtonReleased)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    sf::Vector2i _mouse_pos{sf::Mouse::getPosition(window)};
                    sf::Vector2f mouse_pos = window.mapPixelToCoords(_mouse_pos);
                    points.push_back(point{mouse_pos.x, height - mouse_pos.y, 0});
                    recalculate_path = true;
                }
                else if (event.mouseButton.button == sf::Mouse::Right)
                {

                    selected_point = -1;
                }
            }
            else if (event.type == sf::Event::MouseButtonPressed)
            {
                if (event.mouseButton.button == sf::Mouse::Right)
                {
                    selected_point = closest_point;
                    if (selected_point != -1)
                    {
                        point p{points.at(selected_point)};
                        sf::Vector2i _mouse_pos{sf::Mouse::getPosition(window)};
                        sf::Vector2f mouse_pos = window.mapPixelToCoords(_mouse_pos);
                        dx_to_selected_point = p.x_pos - mouse_pos.x;
                        dy_to_selected_point = p.y_pos - (height - mouse_pos.y);
                    }
                }
            }
        }

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::P) && selected_point != -1)
        {
            points.erase(points.begin() + selected_point);
            selected_point = -1;
        }
                // Check for nearby points
        closest_point = -1;
        for (unsigned int i{0}; i < points.size(); ++i)
        {
            point p{points.at(i)};
            sf::Vector2i _mouse_pos{sf::Mouse::getPosition(window)};
            sf::Vector2f mouse_pos = window.mapPixelToCoords(_mouse_pos);
            float distance{static_cast<float>(hypot(p.x_pos - mouse_pos.x, p.y_pos - (height - mouse_pos.y)))};
            if (distance < point_selection_radius)
            {
                // Point is close enough to be selected but there might be more than
                // one point that is close enough so we have to save the distance to this
                // point and check if there are any other points that are closer.
                // The closest point will be chosen
                if (closest_point == -1)
                {
                    // No other point has been found before that is close enough
                    // so we can simply set the closest point to be this point
                    // without checking the distance
                    closest_point = i;
                    distance_to_closest_point = distance;
                }
                else
                {
                    // Another close point has already been found
                    // so we have to check if this new point is closer
                    if (distance < distance_to_closest_point)
                    {
                        closest_point = i;
                        distance_to_closest_point = distance;
                    }
                }
            }
        }

        // Rotate the closest point if the left or right arrow key is pressed
        if (closest_point != -1)
        {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
            {
                points.at(closest_point).original_angle += rotation_speed;
                if (points.at(closest_point).original_angle >= 360 * 60)
                    points.at(closest_point).original_angle -= 360 * 60;
                recalculate_path = true;
            }
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
            {
                points.at(closest_point).original_angle -= rotation_speed;
                if (points.at(closest_point).original_angle < 0)
                    points.at(closest_point).original_angle += 360 * 60;
                recalculate_path = true;
            }
        }
        // Increase/decrease rotation speed
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
        {
            rotation_speed += 1;
            if (rotation_speed > 250)
                rotation_speed = 250;
        }
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
        {
            rotation_speed -= 1;
            if (rotation_speed < 1)
                rotation_speed = 1;
        }

        // Move the selected point if there is one
        if (selected_point != -1)
        {
            sf::Vector2i _mouse_pos{sf::Mouse::getPosition(window)};
            sf::Vector2f mouse_pos = window.mapPixelToCoords(_mouse_pos);
            points.at(selected_point).x_pos = mouse_pos.x + dx_to_selected_point;
            points.at(selected_point).y_pos = (height - mouse_pos.y) + dy_to_selected_point;
            recalculate_path = true;
        }

        // Generate path and measure the time it takes
        if (recalculate_path)
        {
            std::chrono::steady_clock::time_point time_before{std::chrono::steady_clock::now()};
            path_points = calculate_path(points);
            std::chrono::steady_clock::time_point time_after{std::chrono::steady_clock::now()};
            time_us = std::chrono::duration_cast<std::chrono::microseconds>(time_after - time_before).count();
            recalculate_path = false;
        }
        // Find the closest path point
        closest_path_point = -1;
        for (unsigned int i{0}; i < path_points.size(); ++i)
        {
            path_point p{path_points.at(i)};
            sf::Vector2i _mouse_pos{sf::Mouse::getPosition(window)};
            sf::Vector2f mouse_pos = window.mapPixelToCoords(_mouse_pos);
            float distance{static_cast<float>(hypot(p.x_pos - mouse_pos.x, p.y_pos - (height - mouse_pos.y)))};
            if (distance < path_point_selection_radius)
            {
                if (closest_point == -1)
                {
                    closest_path_point = i;
                    distance_to_closest_path_point = distance;
                }
                else
                {
                    if (distance < distance_to_closest_path_point)
                    {
                        closest_path_point = i;
                        distance_to_closest_path_point = distance;
                    }
                }
            }
        }

        // Draw
        window.clear();
        // Rita ut 6x6 meter
        sf::RectangleShape linjal{sf::Vector2f{8000, 8000}};
        linjal.setPosition(sf::Vector2f{-4000, -4000});
        linjal.setOutlineThickness(10);
        linjal.setFillColor(sf::Color::Transparent);;;;;;;;;;;
        linjal.setOutlineColor(sf::Color::Red);
        window.draw(linjal);
        // Draw the path
        for (unsigned int i{0}; i < path_points.size(); ++i)
        {
            path_point p{path_points.at(i)};
            sf::CircleShape c{150};
            c.setOrigin(c.getRadius(), c.getRadius());
            c.setPosition(p.x_pos, height - p.y_pos);
            //sf::Uint8 cos_value{static_cast<sf::Uint8>(127 * (1 + cos(frame_counter / 10.0)))};
            //sf::Color color{segment_colors.at((i / NUM_POINTS_PER_SEGMENT) % segment_colors.size())};
            float radius{1.0 / abs(p.curvature)};
            int red{255 - radius / 10};
            if (red > 255)
                red = 255;
            if (red < 0)
                red = 0;
            sf::Color color{red, 255 - red, 0};
            if (i == closest_path_point)
                color = sf::Color::Blue;
            c.setFillColor(color);
            window.draw(c);

        }
        // Draw the points
        // Each point's angle is also shown using a line from the center
        // of the circle
        for (unsigned int i{0}; i < points.size(); ++i)
        {
            point p{points.at(i)};

            sf::CircleShape cee{80};
            cee.setFillColor(sf::Color::Red);
            cee.setOrigin(cee.getRadius(), cee.getRadius());
            cee.setPosition(p.x_pos + 425 * cos(p.original_angle / 60.0 / 180 * 3.1415 + 3.1415 / 2), height - p.y_pos - 425 * sin(p.original_angle / 60.0 / 180 * 3.1415 + 3.1415 / 2));
            window.draw(cee);

            cee.setFillColor(sf::Color::Red);
            cee.setPosition(p.x_pos- 425 * cos(p.original_angle / 60.0 / 180 * 3.1415 + 3.1415 / 2), height - p.y_pos + 425 * sin(p.original_angle / 60.0 / 180 * 3.1415 + 3.1415 / 2));
            window.draw(cee);

            /*sf::CircleShape circle{80};
            sf::Color color{sf::Color::Green};
            if (i == closest_point)
                color = sf::Color::Blue;
            if (i == selected_point)
                color = sf::Color::Red;
            circle.setFillColor(color);
            circle.setPosition(p.x_pos - circle.getRadius(), height - p.y_pos - circle.getRadius());
            window.draw(circle);*/
            sf::RectangleShape line{sf::Vector2f{200, 30}};
            line.setRotation(-(p.original_angle / 60));
            line.setPosition(p.x_pos, height - p.y_pos);
            line.setFillColor(sf::Color::Green);
            window.draw(line);

            sf::Text text{std::to_string(i), font};
            text.setCharacterSize(200);
            text.setFillColor(sf::Color::Blue);
            text.setPosition(p.x_pos, -p.y_pos);
            window.draw(text);
        }

        //Rita ut närmsta punkten
        path_point _closest_point{closest_point_to_car(path_points)};
        sf::CircleShape ccp{150};
        ccp.setPosition(_closest_point.x_pos,
            height - _closest_point.y_pos);
        ccp.setFillColor(sf::Color::Yellow);
        ccp.setOrigin(ccp.getRadius(), ccp.getRadius());
        window.draw(ccp);

        //Rita ut bilens postion
        sf::RectangleShape rectcarpos{sf::Vector2f{420, 300}};
        rectcarpos.setPosition(car_x_pos,
            height - car_y_pos);
        rectcarpos.setFillColor(sf::Color::Magenta);
        rectcarpos.setOrigin(rectcarpos.getSize().x / 2, rectcarpos.getSize().y / 2);
        rectcarpos.setRotation(car_angle /60.0);
        window.draw(rectcarpos);

        window.setView(old_view);

        // Show info about the closest point or the selected point
        int closest_point_text_drawn{0};
        if (selected_point != -1)
        {
            point p{points.at(selected_point)};

            sf::Text title_text{};
            title_text.setFont(font);
            title_text.setString("Selected point:");
            title_text.setCharacterSize(42);
            title_text.setFillColor(sf::Color::Red);
            title_text.setPosition(10, 10);
            title_text.setStyle(sf::Text::Bold | sf::Text::Underlined);
            window.draw(title_text);

            sf::Text info_text{};
            info_text.setFont(font);
            std::ostringstream info_text_stream{};
            info_text_stream << "x:     " << p.x_pos << '\n';
            info_text_stream << "y:     " << p.y_pos << '\n';
            info_text_stream << "angle: " << p.original_angle << "\" (" << (p.original_angle / 60.0) << " deg)\n";
            info_text.setString(info_text_stream.str());
            info_text.setCharacterSize(30);
            info_text.setFillColor(sf::Color::Red);
            info_text.setPosition(10, 60);
            window.draw(info_text);

            closest_point_text_drawn = 1;
        }
        else if (closest_point != -1)
        {
            point p{points.at(closest_point)};

            sf::Text title_text{};
            title_text.setFont(font);
            title_text.setString("Closest point:");
            title_text.setCharacterSize(42);
            title_text.setFillColor(sf::Color::Red);
            title_text.setPosition(10, 10);
            title_text.setStyle(sf::Text::Bold | sf::Text::Underlined);
            window.draw(title_text);

            sf::Text info_text{};
            info_text.setFont(font);
            std::ostringstream info_text_stream{};
            info_text_stream << "x:     " << p.x_pos << '\n';
            info_text_stream << "y:     " << p.y_pos << '\n';
            info_text_stream << "angle: " << p.original_angle << "\" (" << (p.original_angle / 60.0) << " deg)\n";
            info_text.setString(info_text_stream.str());
            info_text.setCharacterSize(30);
            info_text.setFillColor(sf::Color::Red);
            info_text.setPosition(10, 60);
            window.draw(info_text);

            closest_point_text_drawn = 1;
        }

        // Show info about the closest path point
        if (closest_path_point != -1)
        {
            path_point p{path_points.at(closest_path_point)};

            sf::Text title_text{};
            title_text.setFont(font);
            title_text.setString("Closest path point:");
            title_text.setCharacterSize(42);
            title_text.setFillColor(sf::Color::Red);
            title_text.setPosition(10 + closest_point_text_drawn * 490, 10);
            title_text.setStyle(sf::Text::Bold | sf::Text::Underlined);
            window.draw(title_text);

            sf::Text info_text{};
            info_text.setFont(font);
            std::ostringstream info_text_stream{};
            info_text_stream << "x:      " << p.x_pos << '\n';
            info_text_stream << "y:      " << p.y_pos << '\n';
            info_text_stream << "radius: " << (1 / p.curvature) << "\n";
            info_text.setString(info_text_stream.str());
            info_text.setCharacterSize(30);
            info_text.setFillColor(sf::Color::Red);
            info_text.setPosition(10 + closest_point_text_drawn * 490, 60);
            window.draw(info_text);
        }

        // Show how many points there are
        sf::Text num_points_text{};
        num_points_text.setFont(font);
        std::ostringstream num_points_text_stream{};
        num_points_text_stream << points.size() << " points";
        num_points_text.setString(num_points_text_stream.str());
        num_points_text.setCharacterSize(30);
        num_points_text.setFillColor(sf::Color::Red);
        num_points_text.setPosition(10, height - 100);
        window.draw(num_points_text);

        // Show how much time the path generation took
        sf::Text time_text{};
        time_text.setFont(font);
        std::ostringstream time_text_stream{};
        time_text_stream << time_us / 1000.0 << " ms";
        time_text.setString(time_text_stream.str());
        time_text.setCharacterSize(30);
        time_text.setFillColor(sf::Color::Red);
        time_text.setPosition(500, height - 100);
        window.draw(time_text);

        // Show angle turning speed
        sf::Text angle_speed_text{};
        angle_speed_text.setFont(font);
        std::ostringstream angle_speed_text_stream{};
        angle_speed_text_stream << rotation_speed << "\t" << calculate_sum_of_squared_curvatures(path_points);// << "\t" <<  minimum_k;
        angle_speed_text.setString(angle_speed_text_stream.str());
        angle_speed_text.setCharacterSize(30);
        angle_speed_text.setFillColor(sf::Color::Red);
        angle_speed_text.setPosition(990, height - 100);
        window.draw(angle_speed_text);

        window.display();

        // Delay for ~60FPS
        if (frame_delay_time > game_clock.getElapsedTime())
        {
          sleep(frame_delay_time - game_clock.getElapsedTime());
        }
        ++frame_counter;
    }
    return 0;
}

// En kurwa mellan varje par av portar med gradient lika med portens riktning
/*
std::vector<path_point> calculate_path(std::vector<point> &through_points)
{
    std::vector<path_point> path_points{};
    through_points = best_angles(through_points);

        // Each pair of points is connected by a curve parametrised by t where t goes from 0 to 1
        for (unsigned int i{0}; i < through_points.size(); ++i)
        {
            std::vector<path_point> points{};
            std::vector<path_point> best_points{};

            float minimum_total_curvature{INFINITY};
            int k_for_minimum_curvature{0};
            int angle_for_best_curvature{through_points.at(i).original_angle};

            //int old_angle{through_points.at(i).best_angle};

            int old_k{1000};
            if (i != 0)
            {
                point current_point{through_points.at(i)};
                old_k = current_point.k;
            }
            point current_point{through_points.at(i)};
            point next_point{through_points.at((i + 1) % through_points.size())};

            int angle{current_point.original_angle};
            int next_angle{next_point.original_angle};
            {
                for (int k{100}; k < 10000; k += 10)
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
                    int e_x = old_k * cos(angle / 60.0 / 180 * 3.1415);
                    int e_y = old_k * sin(angle / 60.0 / 180 * 3.1415);
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
                        path_point new_point{x_pos, y_pos, dx_dt, dy_dt, d2x_dt2, d2y_dt2, curvature};
                        points.push_back(new_point);
                    }

                    double sum_square_curvature{calculate_sum_of_squared_curvatures(points)};
                    double max_curvature{calculate_maximum_curvature(points)};
                    double min_radius{1 / max_curvature};
                    if (sum_square_curvature < minimum_total_curvature && min_radius > 1000)
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
            //if (best_points.size() == 0)
            //{
            //    std::cout << "Hittade ingen väg" << std::endl;
            //}
            //else
            //{
            //    std::cout << "Hittade väg:\tk: " << k_for_minimum_curvature << "\twinkel: (degar) " << (angle_for_best_curvature / 60.0) << std::endl;
            //}
            // Spara valt k och vinkel i portpunkten
            through_points.at((i + 1) % through_points.size()).k = k_for_minimum_curvature;
            through_points.at((i + 1) % through_points.size()).best_angle = angle_for_best_curvature;
            path_points.insert(path_points.end(), best_points.begin(), best_points.end());
        }
    return path_points;
}*/


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
                        path_point new_point{x_pos, y_pos, dx_dt, dy_dt, d2x_dt2, d2y_dt2, curvature};
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

double abs(double x)
{
    return (x < 0) ? -x : x;
}

// I bågminuter
int calculate_angle(int dx, int dy)
{
    float angle{atan2(dy, dx)};
    return angle * 180 / 3.1415 * 60;
}


// Vinklar i bågminuter
/*float clamp_angle(float ang, float min, float max)
{
    float n_min = normalize180((min - ang) / 60.0);
    float n_max = normalize180((max - ang) / 60.0);

    if (n_min <= 0 && n_max >= 0)
    {
        return ang;
    }
    if (abs(n_min) < abs(n_max))
        return min;
    return max;
}*/

float calculate_angle_difference(float first_angle, float second_angle)
  {
        float difference = second_angle - first_angle;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return difference;
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

bool is_foscar_a_small_yellow_rubber_duck()
{
    return true;
}

bool a_closer_than_b(path_point a, path_point b)
{
    float da{hypot(a.x_pos - car_x_pos, a.y_pos - car_y_pos)};
    float db{hypot(b.x_pos - car_x_pos, b.y_pos - car_y_pos)};
    return (da < db);
}

path_point closest_point_to_car(std::vector<path_point> points)
{
    int number_of_points{points.size()};
    int look_ahead{50};
    std::vector<path_point> closest_points{};
    for (path_point p : points)
    {
        std::cout << p.x_pos << ',' << p.y_pos << std::endl;
    }
    std::cout << std::endl;
    if (number_of_points > last_point + look_ahead && last_point > look_ahead)
    {
        for (int i{last_point - look_ahead} ; i < last_point + look_ahead ; i++)
        {
            closest_points.push_back(points.at(i));
        }
        std::sort (closest_points.begin(), closest_points.end(), a_closer_than_b);
    }
    else if (last_point < look_ahead)
    {
        for (int i{number_of_points - look_ahead + last_point} ; i < number_of_points ; i++)
        {
            closest_points.push_back(points.at(i));
        }
        for (int i{0} ; i < last_point + look_ahead ; i++)
        {
            closest_points.push_back(points.at(i));
        }
        std::sort (closest_points.begin(), closest_points.end(), a_closer_than_b);
    }
    else
    {
        for (int i{last_point - look_ahead} ; i < number_of_points ; i++)
        {
            closest_points.push_back(points.at(i));
        }
        for (int j{0} ; (j < look_ahead - (number_of_points - last_point))
            && (j < number_of_points - (number_of_points - last_point)) ; j++)
        {
            closest_points.push_back(points.at(j));
        }
        std::sort (closest_points.begin(), closest_points.end(), a_closer_than_b);

    }
    /*for (point p : closest_points)
    {
        std::cout << p.x_pos << ',' << p.y_pos << std::endl;
    }
    std::cout << closest_points.size() << std::endl;*/
    return closest_points.at(0);
}
