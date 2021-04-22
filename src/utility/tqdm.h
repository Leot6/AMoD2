#ifndef TQDM_H
#define TQDM_H
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <numeric>
#include <ios>
#include <string>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <libc.h>
#include <sys/ioctl.h>
#undef NDEBUG
#include <assert.h>

class tqdm {
    private:
        // time, iteration counters and deques for rate calculations
        std::chrono::time_point<std::chrono::system_clock> t_first = std::chrono::system_clock::now();
        std::chrono::time_point<std::chrono::system_clock> t_old = std::chrono::system_clock::now();
        int n_old = 0;
        std::vector<double> deq_t;
        std::vector<int> deq_n;
        int nupdates = 0;
        int total_iterations = 0;
        int period = 1;
        unsigned int smoothing = 50;
        bool use_ema = false;
        float alpha_ema = 0.1;

        std::vector<const char*> bars = {" ", "▏", "▎", "▍", "▌", "▋", "▊", "▉", "█"};

        bool in_screen = (system("test $STY") == 0);
        bool in_tmux = (system("test $TMUX") == 0);
        bool is_tty = isatty(1);
        int width = 30;
        int width_adjustment = 40;
        std::string label = "";
        int current_iteration = 0;

    public:
        tqdm(std::string label_, int total_iterations_) {
            label = label_;
            total_iterations = total_iterations_;
            set_theme_braille();
        }

        void reset() {
            t_first = std::chrono::system_clock::now();
            t_old = std::chrono::system_clock::now();
            n_old = 0;
            deq_t.clear();
            deq_n.clear();
            period = 1;
            nupdates = 0;
            total_iterations = 0;
            current_iteration = 0;
            label = "";
        }

        void set_theme_line() { bars = {"─", "─", "─", "╾", "╾", "╾", "╾", "━", "═"}; }
        void set_theme_circle() { bars = {" ", "◓", "◑", "◒", "◐", "◓", "◑", "◒", "#"}; }
        void set_theme_braille() { bars = {" ", "⡀", "⡄", "⡆", "⡇", "⡏", "⡟", "⡿", "⣿" }; }
        void set_theme_braille_spin() { bars = {" ", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠇", "⠿" }; }
        void set_theme_vertical() { bars = {"▁", "▂", "▃", "▄", "▅", "▆", "▇", "█", "█"}; }
        void set_theme_basic() {bars = {" ", " ", " ", " ", " ", " ", " ", " ", "#"};
        }
        void set_label(std::string label_) { label = label_; }

        void finish() {
            current_iteration = total_iterations;
            progress();
            printf("\n");
            fflush(stdout);
        }
        void progress() {
            current_iteration++;
            struct winsize size;
            ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
            int window_width = size.ws_col;
            width = window_width - width_adjustment - label.size();
            if (width < 0 || width > 20) {
                width = 30;
            }

            if(is_tty && (current_iteration%period == 0)) {
                nupdates++;
                auto now = std::chrono::system_clock::now();
                double dt = ((std::chrono::duration<double>)(now - t_old)).count();
                double dt_tot = ((std::chrono::duration<double>)(now - t_first)).count();
                int dn = current_iteration - n_old;
                n_old = current_iteration;
                t_old = now;
                if (deq_n.size() >= smoothing) deq_n.erase(deq_n.begin());
                if (deq_t.size() >= smoothing) deq_t.erase(deq_t.begin());
                deq_t.push_back(dt);
                deq_n.push_back(dn);

                double avgrate = 0.;
                if (use_ema) {
                    avgrate = deq_n[0] / deq_t[0];
                    for (unsigned int i = 1; i < deq_t.size(); i++) {
                        double r = 1.0*deq_n[i]/deq_t[i];
                        avgrate = alpha_ema*r + (1.0-alpha_ema)*avgrate;
                    }
                } else {
                    double dtsum = std::accumulate(deq_t.begin(),deq_t.end(),0.);
                    int dnsum = std::accumulate(deq_n.begin(),deq_n.end(),0.);
                    avgrate = dnsum/dtsum;
                }

                // learn an appropriate period length to avoid spamming stdout
                // and slowing down the loop, shoot for ~25Hz and smooth over 3 seconds
                if (nupdates > 10) {
                    period = (int)( std::min(std::max((1.0/25)*current_iteration/dt_tot,1.0), 5e5));
                    smoothing = 25*3;
                }
                double peta = (total_iterations-current_iteration)/avgrate;
                double pct = (double)current_iteration/(total_iterations*0.01);
                if( ( total_iterations - current_iteration ) <= period ) {
                    pct = 100.0;
                    avgrate = total_iterations/dt_tot;
                    current_iteration = total_iterations;
                    peta = 0;
                }

                int time_consumed_s = dt_tot;
                int time_consumed_min = time_consumed_s / 60;
                time_consumed_s %= 60;
                int time_consumed_hour = time_consumed_min / 60;
                time_consumed_min %= 60;
                int time_remaining_s = peta;
                int time_remaining_min = time_remaining_s / 60;
                time_remaining_s %= 60;
                int time_remaining_hour = time_remaining_min / 60;
                time_remaining_min %= 60;
                if (time_consumed_hour > 0 || time_remaining_hour > 0) { width -= 6; }

                std::string unit = "Hz";
                double div = 1.;
                if (avgrate > 1e6) {
                    unit = "MHz"; div = 1.0e6;
                } else if (avgrate > 1e3) {
                    unit = "kHz"; div = 1.0e3;
                }

                double fills = ((double)current_iteration / total_iterations * width);
                int ifills = (int)fills;

                printf("\015 ");
                // label
                printf("%s:", label.c_str());
                // percentage
                printf("%4.1f%% ", pct);
                // bar
                for (int i = 0; i < ifills; i++) { std::cout << bars[8]; }
                if (!in_screen && (current_iteration != total_iterations)) { printf("%s",bars[(int)(8.0*(fills-ifills))]); }
                for (int i = 0; i < width-ifills-1; i++) { std::cout << bars[0]; }
                // count
                printf("%d/%d ", current_iteration, total_iterations);
                // time and speed
                if (time_consumed_hour > 0 || time_remaining_hour > 0) {
                    printf("[%02d:%02d:%02d<%02d:%02d:%02d|%3.1f%s]",
                           time_consumed_hour, time_consumed_min, time_consumed_s,
                           time_remaining_hour, time_remaining_min, time_remaining_s,
                           avgrate/div, unit.c_str());
                } else {
                    printf("[%02d:%02d<%02d:%02d|%3.1f%s]",
                           time_consumed_min, time_consumed_s,
                           time_remaining_min, time_remaining_s,
                           avgrate/div, unit.c_str());
                }

                if ((total_iterations - current_iteration) > period) { fflush(stdout); }

            }
        }
};
#endif
