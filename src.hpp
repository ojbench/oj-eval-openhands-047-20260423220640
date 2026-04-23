#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"
#include <algorithm>
#include <cmath>

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    /////////////////////////////////
    /// TODO: You can add any [private] member variable or [private] member function you need.
    /////////////////////////////////
    
    bool will_collide(const Vec &my_v, const Vec &other_v, const Vec &other_pos, double other_r) {
        Vec delta_pos = pos_cur - other_pos;
        Vec delta_v = my_v - other_v;
        
        double project = delta_pos.dot(delta_v);
        if (project >= 0) {
            return false;
        }
        
        double dv_norm = delta_v.norm();
        if (dv_norm < 1e-9) {
            return delta_pos.norm_sqr() < (r + other_r) * (r + other_r);
        }
        
        project /= -dv_norm;
        double min_dis_sqr;
        double delta_r = r + other_r;
        
        if (project < dv_norm * 0.1) {
            min_dis_sqr = delta_pos.norm_sqr() - project * project;
        } else {
            min_dis_sqr = (delta_pos + delta_v * 0.1).norm_sqr();
        }
        
        return min_dis_sqr <= delta_r * delta_r - 0.01;
    }

public:

    Vec get_v_next() {
        Vec to_target = pos_tar - pos_cur;
        double dist_to_target = to_target.norm();
        
        if (dist_to_target < 0.01) {
            return Vec(0, 0);
        }
        
        Vec desired_dir = to_target.normalize();
        double desired_speed = std::min(v_max, dist_to_target / 0.1);
        Vec desired_v = desired_dir * desired_speed;
        
        int n = monitor->get_robot_number();
        
        bool has_collision = false;
        for (int i = 0; i < n; ++i) {
            if (i == id) continue;
            Vec other_pos = monitor->get_pos_cur(i);
            Vec other_v = monitor->get_v_cur(i);
            double other_r = monitor->get_r(i);
            if (will_collide(desired_v, other_v, other_pos, other_r)) {
                has_collision = true;
                break;
            }
        }
        
        if (!has_collision) {
            return desired_v;
        }
        
        for (double speed_ratio = 1.0; speed_ratio >= 0.1; speed_ratio -= 0.1) {
            for (int angle = 5; angle <= 90; angle += 5) {
                double rad = angle * 3.14159265358979323846 / 180.0;
                double speed = desired_speed * speed_ratio;
                if (speed > v_max) speed = v_max;
                
                Vec v1 = Vec(desired_dir.x * cos(rad) - desired_dir.y * sin(rad),
                             desired_dir.x * sin(rad) + desired_dir.y * cos(rad)) * speed;
                Vec v2 = Vec(desired_dir.x * cos(-rad) - desired_dir.y * sin(-rad),
                             desired_dir.x * sin(-rad) + desired_dir.y * cos(-rad)) * speed;
                
                bool v1_ok = true, v2_ok = true;
                for (int i = 0; i < n; ++i) {
                    if (i == id) continue;
                    Vec other_pos = monitor->get_pos_cur(i);
                    Vec other_v = monitor->get_v_cur(i);
                    double other_r = monitor->get_r(i);
                    
                    if (v1_ok && will_collide(v1, other_v, other_pos, other_r)) {
                        v1_ok = false;
                    }
                    if (v2_ok && will_collide(v2, other_v, other_pos, other_r)) {
                        v2_ok = false;
                    }
                    if (!v1_ok && !v2_ok) break;
                }
                
                if (v1_ok) return v1;
                if (v2_ok) return v2;
            }
        }
        
        return Vec(0, 0);
    }
};


/////////////////////////////////
/// TODO: You can add any class or struct you need.
/////////////////////////////////


#endif //PPCA_SRC_HPP