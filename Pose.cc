#include "Pose.h"
#include "gottochblandat.h"
using namespace std;


Pose::Pose(float _x_pos, float _y_pos, float _angle)
{
    sens_data.mu = VectorXf::Zero(3);
    sens_data.mu(0) = _x_pos;
    sens_data.mu(1) = _y_pos;
    sens_data.mu(2) = _angle;
    cov = MatrixXf::Zero(3,3);
    R = MatrixXf::Zero(3,3);
}

MatrixXf Pose::G(float vel, float dt, float steer_angle, float lr, float lf)
{
    float w{vel / lr * static_cast<float>
        (sin(atan(tan(steer_angle * 3.1415 / 60.0 / 180.0)* lr / (lf + lr))))};

    MatrixXf G = MatrixXf::Identity(3,3);
    G(0,2) = -vel * cosc_ish(sens_data.mu(2) * 3.1415 / 60.0 / 180.0, w, dt / 1000.0);
    G(1,2) =  vel * sinc_ish(sens_data.mu(2) * 3.1415 / 60.0 / 180.0, w, dt / 1000.0);
    return G;
}

//Jacobianmatris av h givet landmärken i polära koordinater
MatrixXf Pose::H(float dx, float dy, float q)
{
    MatrixXf H = MatrixXf::Zero(2, 5);
    H(0,0) = -sqrt(q)*dx;  H(0,1) = -sqrt(q)*dy;    H(0,2) = 0;    H(0,3) = sqrt(q)*dx;    H(0,4) = sqrt(q)*dy;
    H(1,0) = dy;   H(1,1) = -dx;  H(1,2) = -q;    H(1,3) = -dy;    H(1,4) = dx;
    return 1/q * H;
}

// uppskattar en ny position givet fordonsmodellen
VectorXf Pose::estimate_new_pose(float vel, float steer_angle, float dt, float lr,
float lf)
{
    float w{vel / lr * static_cast<float>
        (sin(atan(tan(steer_angle * 3.1415 / 180.0 / 60.0)* lr / (lf + lr))))};
    VectorXf new_pose(3);
    new_pose(0) = sens_data.mu(0) + vel * sinc_ish(sens_data.mu(2) * 3.1415 / 180 / 60, w, dt / 1000.0);
    new_pose(1) = sens_data.mu(1) + vel * cosc_ish(sens_data.mu(2) * 3.1415 / 180 / 60, w, dt / 1000.0);
    new_pose(2) = normalize(sens_data.mu(2) + w * 60 *180 / 3.14159265359 * dt / 1000.0);

    return new_pose;
}

// Utför SLAM
void Pose::SLAM(float vel, float steer_angle, float dt,float lr,float lf)
{
    int mat_dim{2 * amount_landm + 3};
    float to_radians {3.14159265359/60.0/180.0};
    float to_minute_of_arc{60.0*180.0/3.14159265359};

// Uppskatta en ny position
    sens_data.mu.block(0,0,3,1) = estimate_new_pose(vel, steer_angle, dt, lr, lf);

// Uppskatta ny kovarians
    MatrixXf G_mat = G(vel, dt, steer_angle, lr, lf);
    cov.block(0,0,3,3) = G_mat * cov.block(0,0,3,3) * G_mat.transpose();

// felet i fordonsmodellen
    R(0,0) = dt/2; //Justerbar
    R(1,1) = dt/2; //Justerbar
    R(2,2) = 10 * dt; //Justerbar
    cov = cov + R;

 
    for (int i{}; i < ((sens_data.obs_mu.rows() - 3) / 2); i++)
    {
        // observerad position för landmärket i ursprungliga koordinatsystemet
        // och koordinatsystemet utgående från bilen
        float obs_x{sens_data.obs_mu(2 * i + 3)};
        float obs_y{sens_data.obs_mu(2 * i + 4)};
        if((obs_x != 0) && (obs_y != 0) && (sens_data.mu_info(2 * i + 4) > 5))
        {

            int cur_landm_index{2 * i + 3};
            
            float obs_x_inicoord{obs_x * cos(sens_data.mu(2) * to_radians) -
                obs_y * sin(sens_data.mu(2)* to_radians) + sens_data.mu(0)};
            float obs_y_inicoord{obs_x * sin(sens_data.mu(2) * to_radians) +
                obs_y * cos(sens_data.mu(2)* to_radians) + sens_data.mu(1)};
    
            float obs_r{sqrt(obs_x*obs_x + obs_y * obs_y)};
            float obs_phi{normalize((atan2(obs_y,obs_x)) * to_minute_of_arc)};

            // koordinaten landmärket borde finnas på enligt fordonsmodellen
            float pred_r{sqrt((sens_data.mu(cur_landm_index)-sens_data.mu(0))*(sens_data.mu(cur_landm_index)-sens_data.mu(0)) +
                (sens_data.mu(cur_landm_index + 1)-sens_data.mu(1))*(sens_data.mu(cur_landm_index + 1)-sens_data.mu(1)))};
            float pred_phi{(atan2((sens_data.mu(cur_landm_index + 1)-sens_data.mu(1)),(sens_data.mu(cur_landm_index)-sens_data.mu(0)))) *
                to_minute_of_arc - sens_data.mu(2)};
            pred_phi = normalize(pred_phi);

            // felet i mätningen
            MatrixXf Q = MatrixXf::Zero(2,2);
            Q(0,0) = 0.01*obs_r; //justerbar
            Q(1,1) = 2.5/3; // justerbar
    
            MatrixXf H_mat = H((sens_data.mu(cur_landm_index)-sens_data.mu(0)), (sens_data.mu(cur_landm_index + 1)-sens_data.mu(1)), pred_r*pred_r);
    
            // skapar en ny rad/kollonn i alla matriser och vektorer om landmärket
            // inte observerats tidigare
            if(i >= amount_landm)
            {
                amount_landm++;
                mat_dim = 2 * amount_landm + 3;
                cur_landm_index = mat_dim - 2;
                R.conservativeResizeLike(MatrixXf::Zero(mat_dim, mat_dim));
                cov.conservativeResizeLike(MatrixXf::Zero(mat_dim, mat_dim));
                
                sens_data.mu(cur_landm_index) = obs_x_inicoord;
                sens_data.mu(cur_landm_index + 1) = obs_y_inicoord;
                cov(cur_landm_index, cur_landm_index) = 200;
                cov(cur_landm_index + 1, cur_landm_index + 1) = 200;
                pred_r = obs_r;
                pred_phi = obs_phi;
                H_mat = H(obs_x, obs_y, pred_r*pred_r); //Q_k();
            }
            MatrixXf Fx = MatrixXf::Zero(5, mat_dim);
            Fx.block(0,0,3,3) = MatrixXf::Identity(3,3);
            Fx.block(3, cur_landm_index, 2, 2) = MatrixXf::Identity(2,2);
            MatrixXf Gt = H_mat * Fx;
    
            // beräknar kalmanförstärkningen
            MatrixXf Kalman_gain = cov*Gt.transpose()*(Gt*cov * Gt.transpose() + Q).inverse();
    
            MatrixXf h_mu = MatrixXf::Zero(2,1); //förväntad mätning
            h_mu(0,0) = pred_r;
            h_mu(1,0) = pred_phi;
    
            //Beräkna ny medelvärdesvektorn
            MatrixXf z = MatrixXf::Zero(2,1); // faktisk mätning
            z(0,0) = obs_r;
            z(1,0) = obs_phi;
    
            VectorXf delta_mu{Kalman_gain*(z - h_mu)};
            // om rörelsen ej blir diskontinuerlig
            if(sqrt(pow(delta_mu(0),2) +
                pow(delta_mu(1),2)) < 666)
            {
                // väger samman förväntat och uppmät med kalmanförstärkningen
                sens_data.mu = sens_data.mu + Kalman_gain*(z - h_mu);
                sens_data.mu(2) = normalize(sens_data.mu(2));
                //Beräkna ny kovariansmatrs
                MatrixXf I = MatrixXf::Identity(mat_dim,mat_dim);
                cov = (I - Kalman_gain * Gt) * cov;
            }
        }
    }
}

const VectorXf& Pose::get_mu()
{
    return sens_data.mu;
}

int Pose::get_number_of_landmarks()
{
    return amount_landm;
}

vector<int> Pose::get_landmarks()
{
    vector<int> landms{};

    for(int i{3}; i < 2 * amount_landm + 3; i++)
    {
        landms.push_back(static_cast<int>(sens_data.mu(i)));
    }
    return landms;
}

int Pose::get_x()
{
    return static_cast<int>(sens_data.mu(0));
}
int Pose::get_y()
{
    return static_cast<int>(sens_data.mu(1));
}
int Pose::get_ang()
{
    return static_cast<int>(sens_data.mu(2));
}
/*
void Pose::localization(float vel, float steer_angle, float dt,float lr,float lf)
{
    int mat_dim{2 * amount_landm + 3};
    float to_radians {3.14159265359/60.0/180.0};
    float to_minute_of_arc{60.0*180.0/3.14159265359};
    float sum_x;
    float sum_y;

// Uppskatta en ny position
    mu.block(0,0,3,1) = estimate_new_pose(vel, steer_angle, dt, lr, lf);

    for (int i{}; i < sens_data.obs_landm ; i++)
    {
        // observerad position för landmärket i ursprungliga koordinatsystemet
        // och koordinatsystemet utgående från bilen
        //2-cur
        float obs_x{sens_data.obs_mu(2 * i + 3)};
        float obs_y{sens_data.obs_mu(2 * i + 4)};
        //bilens x-pos = mu(0)
        //bilens y-pos = mu(1)
        //landmärkets position relativt bilen:
        float obs_x_rel_car{obs_x - mu(0)};
        float obs_y_rel_car{obs_y - mu(1)};

        sum_x += obs_x_rel_car;
        sum_y += obs_y_rel_car;

        float obs_x_inicoord{obs_x * cos(mu(2) * to_radians) -
            obs_y * sin(mu(2)* to_radians) + mu(0)};
        float obs_y_inicoord{obs_x * sin(mu(2) * to_radians) +
            obs_y * cos(mu(2)* to_radians) + mu(1)};

        float obs_r{sqrt(obs_x*obs_x + obs_y * obs_y)};
        float obs_phi{(atan2(obs_y,obs_x)) * to_minute_of_arc};


        // parar ihop observerade med existerande landmärken
        int cur_landm_index = data_assocation(obs_x_inicoord, obs_y_inicoord,
            sens_data.obs_mu(2 * i + 3), mu);

        // koordinaten landmärket borde finnas på enligt fordonsmodellen
        float pred_r{sqrt((mu(cur_landm_index)-mu(0))*(mu(cur_landm_index)-mu(0)) +
            (mu(cur_landm_index + 1)-mu(1))*(mu(cur_landm_index + 1)-mu(1)))};
        float pred_phi{(atan2((mu(cur_landm_index + 1)-mu(1)),(mu(cur_landm_index)-mu(0)))) *
            to_minute_of_arc - mu(2)};
        pred_phi = sens_data.normalize(pred_phi);

        // skapar en ny rad/kollon i alla matriser och vektorer om landmärket
        // inte observerats tidigare
        if(cur_landm_index == 0)
        {
            amount_landm++;
            mat_dim = 2 * amount_landm + 3;
            cur_landm_index = mat_dim - 2;
            R.conservativeResizeLike(MatrixXf::Zero(mat_dim, mat_dim));
            mu.conservativeResizeLike(VectorXf::Zero(mat_dim));
            cov.conservativeResizeLike(MatrixXf::Zero(mat_dim, mat_dim));

            mu(cur_landm_index) = obs_x_inicoord;
            mu(cur_landm_index + 1) = obs_y_inicoord;
            cov(cur_landm_index, cur_landm_index) = 200;
            cov(cur_landm_index + 1, cur_landm_index + 1) = 200;
            pred_r = obs_r;
            pred_phi = obs_phi;
            //H_mat = H(obs_x, obs_y, pred_r*pred_r); //Q_k();

        }
    }
}
void Pose::calculate_mean(float sum_x, float sum_y)
{
    float mean_obs_x{sum_x/(sens_data.obs_landm)};
    float mean_obs_y{sum_y/(sens_data.obs_landm)};
}
*/
