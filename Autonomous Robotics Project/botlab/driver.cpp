#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <cassert>
#include <math.h>
#include <chrono>
#include <fftw3.h>
#include "parameters.h"
#include <signal.h>
#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <csignal>
#include <lcmtypes/pose_xyt_t.hpp>

#include <unistd.h>
#include <cassert>

#include <chrono>
#include <iostream>

#include "occupancy_grid.hpp"


// ------------- TCP/UDP CONFIGURATION -------------

using namespace std;

OccupancyGrid grid;
int init = 1;
int best_mat = 5;
int real_best_mat = 5;

class Handler 
{
    public:
        ~Handler() {}
        int count = 0;
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const pose_xyt_t* pose)
        {
            if(init){
                grid = OccupancyGrid(5.0f, 5.0f, 0.025f);
                grid.setOrigin(pose->x / grid.metersPerCell_, pose->y / grid.metersPerCell_);
                init = 0;
            }
                if(real_best_mat != 5){
                    grid.setMats(pose->x, pose->y, real_best_mat);
                }
            
        }
        
};




struct sockaddr_in server_addr;
const int REUSE_ADDR = 1;

#define PORT 8000

#define LARGE_BUFFER_SIZE 65536

void read_duty();
void read_frequency();

float get_rms();
float mean(float * arr, int len);
float stdev(float * arr, float avg, int len);
float sample_stdev(float * arr, float avg, int len);
float get_dist(float * mat_data, float avg, float std_dev, float rms);
void scale_features();
float get_dist_simple(float mat_avg, float avg);
int majority(int *arr, int len);

float l_duty, r_duty, l_vel, r_vel;
float acc;


int duty_fd, freq_fd;

// #define DUTY_BUFFER_SIZE 100
// #define ACC_BUFFER_SIZE 100


//                          DUTY/VEL,         STDEV,            RMS
float wood[3] = {1.658081604, 0.09587365793, 0.121902011};
float wavy[3] = {1.650032609, 0.2204692434, 0.246376372};
//float hexagon[3]   = {1.6939184, 0.1474898386, 0.2605186563};
float hexagon[3]   = {1.6939184, 0.1474898386, 0.27};

float carpet[3]    = {2.132345455, 0.2502523172, 0.09137843619};
//float blacktile[3] = {1.689234562, 0.08258677173, 0.1718815526};
float blacktile[3] = {1.689234562, 0.08258677173, 0.20};


//const char* material[5] = {"wood", "wavy", "hexagon", "carpet", "unknown"};
const char* material[6] = {"wood", "wavy", "hexagon", "carpet", "blacktile", "unknown"};


char large_buffer[LARGE_BUFFER_SIZE];

float l_duty_buffer[DUTY_BUFFER_SIZE]; //should store duty/vel into buffer
//float r_duty_buffer[DUTY_BUFFER_SIZE];
float acc_buffer[ACC_BUFFER_SIZE];

// feature scaling values
float duty_avg, duty_stdev, stdev_avg, stdev_stdev, rms_avg, rms_stdev; 


//float carpet_duty_simple = 1.864823431;
//float tile_duty_simple = 1.59842964;
 


int main(int argc, char *argv[]){	

    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    
    Handler handlerObject;
    lcm.subscribe("ODOMETRY", &Handler::handleMessage, &handlerObject);

    



    int sample_history[SAMPLE_HIST_SIZE];
    memset(sample_history, 0, SAMPLE_HIST_SIZE*sizeof(int));
    int sample_hist_ind = 0;

    int server_fd, c1_socket, c2_socket, valread;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[1024] = { 0 };

	// Creating socket file descriptor
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	// Forcefully attaching socket to the port 8080
	if (setsockopt(server_fd, SOL_SOCKET,
				SO_REUSEADDR | SO_REUSEPORT, &opt,
				sizeof(opt))) {
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(PORT);

	// Forcefully attaching socket to the port 8080
	if (bind(server_fd, (struct sockaddr*)&address,
			sizeof(address))
		< 0) {
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	if (listen(server_fd, 3) < 0) {
		perror("listen");
		exit(EXIT_FAILURE);
	}
	if ((c1_socket = accept(server_fd, (struct sockaddr*)&address,
				(socklen_t*)&addrlen))
		< 0) {
		perror("accept");
		exit(EXIT_FAILURE);
	}

    printf("accept client1\n");
    
    if ((c2_socket
		= accept(server_fd, (struct sockaddr*)&address,
				(socklen_t*)&addrlen))
		< 0) {
		perror("accept");
		exit(EXIT_FAILURE);
	}

    printf("accept client2\n");

	freq_fd = c1_socket;
    duty_fd = c2_socket;

    fftw_complex *in;
	fftw_complex *out;
	fftw_plan p;

	in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * ACC_BUFFER_SIZE);
	out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * ACC_BUFFER_SIZE);
	p = fftw_plan_dft_1d(ACC_BUFFER_SIZE, in, out, FFTW_FORWARD, FFTW_MEASURE);

    auto start = std::chrono::high_resolution_clock::now();

    scale_features();
    int count = 0;
    float avg_rms = 0;
    int num_samp = 0;

    float prev_min_dist = 0;
    int prev_best_mat = 0;
    //while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count() < 25) {
    while(1){

        
        lcm.handle();


        read_duty(); //assuming this populates buffers accordingly..
        read_frequency();
        best_mat = 5;
        //get means and std dvs of duty and scale
        float l_duty_mean = mean(l_duty_buffer, DUTY_BUFFER_SIZE);

        //float carpet_dist = get_dist_simple(carpet_duty_simple, l_duty_mean);
        
        //float tile_dist = get_dist_simple(tile_duty_simple, l_duty_mean);

        
        //  if(carpet_dist > tile_dist){
        //      printf("tile, %.4f\n", l_duty_mean);
        //  }
        //  else{
        //    printf("carpet, %.4f\n", l_duty_mean);
        // }


        //printf("%.4f\n", l_duty_buffer[99]);
        float l_duty_std_dev = sample_stdev(l_duty_buffer, l_duty_mean, DUTY_BUFFER_SIZE);
        float unscaled_mean = l_duty_mean;
        float unscaled_stdev = l_duty_std_dev;
        
        bool moving = l_duty_mean > 0.0001;
        l_duty_mean =  (l_duty_mean - duty_avg) / duty_stdev; 
        l_duty_std_dev = (l_duty_std_dev - stdev_avg) / stdev_stdev;
        
        // float r_duty_mean = mean(r_duty_buffer, DUTY_BUFFER_SIZE);
        // r_duty_mean =  (r_duty_mean - duty_avg) / duty_stdev;
        //float r_duty_std_dev = stdev(r_duty_buffer, r_duty_mean, DUT_BUFFER_SIZE);

        //rms calc for acc data
        float acc_rms = get_rms();
        float unscaled_rms = acc_rms;
        float acc_std_dev = sample_stdev(acc_buffer, mean(acc_buffer, ACC_BUFFER_SIZE), ACC_BUFFER_SIZE);
        avg_rms = (avg_rms * num_samp + acc_rms) / ((float) ++num_samp);
        //printf("%f %f\n", acc_std_dev, acc_rms);
        acc_rms = (acc_rms - rms_avg) / rms_stdev;

        //fft correlation for acc

        for (size_t i=0; i<ACC_BUFFER_SIZE; ++i) {
            in[i][0] = acc_buffer[i]; // correct for dc bias
            in[i][1] = 0;
        }

        fftw_execute(p);
        //-------------------make predictions---------------------------


        float distances[5];
        
        distances[0] = get_dist(wood, l_duty_mean, l_duty_std_dev, acc_rms);
        distances[1] = get_dist(wavy, l_duty_mean, l_duty_std_dev, acc_rms);
        distances[2] = get_dist(hexagon, l_duty_mean, l_duty_std_dev, acc_rms);
        distances[3] = get_dist(carpet, l_duty_mean, l_duty_std_dev, acc_rms); 
        distances[4] = get_dist(blacktile, l_duty_mean, l_duty_std_dev, acc_rms);
        
        float min_dist = 2.5;
        
        
        for(int i = 0; i < 5; ++i){
            if(distances[i] < min_dist){
                min_dist = distances[i];
                best_mat = i;
            }
        }

        for(int i = 0; i < 5; ++i){
            if(i != best_mat && fabs(distances[i] - min_dist) < 0.25){
                
                //min_dist = distances[i];
                best_mat = 5;
                break;
            }
        }

        sample_history[sample_hist_ind] = best_mat;
        sample_hist_ind = (sample_hist_ind + 1) % SAMPLE_HIST_SIZE;
        count++;

        // best_mat = majority(sample_history, SAMPLE_HIST_SIZE);

            // printf("%s, %.4f, %.4f\n", material[best_mat], l_duty_buffer[0], acc_buffer[0]);
        if (count > 10){
            //printf("%s, %.4f, %.4f, %.4f\n", material[best_mat], l_duty_mean, l_duty_std_dev, acc_rms);
           if (fabs(prev_min_dist - min_dist) > 0.7) {
                best_mat = 5;
           }
           else if(fabs(prev_min_dist - min_dist) < 0.1 && prev_best_mat != best_mat){
                best_mat = 5;
           }
            real_best_mat = best_mat;
           prev_min_dist = min_dist;
           prev_best_mat = best_mat;
           if (best_mat < 5) {
                printf("%s, %.4f, %.4f, %.4f, %.4f, %.4f\n", material[best_mat], distances[0], distances[1], distances[2], distances[3], distances[4]);
                grid.saveToFile_mats("current.map");
           }
           //printf("%s, %.4f, %.4f, %.4f\n", material[best_mat], unscaled_mean, unscaled_stdev, unscaled_rms);
            
            count = 0;
        }
        

        
        // else{
        //     printf("not moving\n");
        // }
        
    }

    std::cout << "finished\n" << std::endl;

	// closing the connected socket
	close(c1_socket);
    close(c2_socket);
	// closing the listening socket
	shutdown(server_fd, SHUT_RDWR);

    return 0;
}

void read_duty() {
    //int n = read(duty_fd, (char*) l_duty_buffer, sizeof(float)*DUTY_BUFFER_SIZE);
    int n = read(duty_fd, (char*)large_buffer, LARGE_BUFFER_SIZE);
    
    if (n > sizeof(float)*DUTY_BUFFER_SIZE) {    
        n = n - (n % sizeof(float));
        memcpy(l_duty_buffer, &large_buffer[(n-sizeof(float)*DUTY_BUFFER_SIZE)], sizeof(float)*DUTY_BUFFER_SIZE);
    }
    else {
        memcpy(l_duty_buffer, large_buffer, n);
    }
    
    // for (int i=0; i<DUTY_BUFFER_SIZE; ++i) {
    //     printf("%f ", l_duty_buffer);
    // }
    // printf("\n");
}

void read_frequency() {
    int n = read(freq_fd, (char*)large_buffer, LARGE_BUFFER_SIZE);
    if (n > sizeof(float)*ACC_BUFFER_SIZE) {    
        n = n - (n % sizeof(float));
        memcpy(acc_buffer, &large_buffer[(n-sizeof(float)*ACC_BUFFER_SIZE)], sizeof(float)*ACC_BUFFER_SIZE);
    }
    else {
        memcpy(acc_buffer, large_buffer, n);
    }
}


float get_rms(){
    float sum = 0;
    for (size_t i=0; i<ACC_BUFFER_SIZE; ++i) {
        sum += acc_buffer[i] * acc_buffer[i];
    }
    sum /= ACC_BUFFER_SIZE;
    return pow(sum, 0.5);
}

// duty_avg, duty_stdev, stdev_avg, stdev_stdev, rms_avg, rms_stdev; 
void scale_features(){
    
    float mat_duty[5] = {wood[0], wavy[0], carpet[0], hexagon[0], blacktile[0]};
    float mat_stdev[5] = {wood[1], wavy[1], carpet[1], hexagon[1], blacktile[1]};
    float mat_rms[5] = {wood[2], wavy[2], carpet[2], hexagon[2], blacktile[2]};

    duty_avg = mean(mat_duty, 5);
    stdev_avg = mean(mat_stdev, 5);
    rms_avg = mean(mat_rms, 5);

    // printf("%.4f, %.4f, %.4f\n", duty_avg, stdev_avg, rms_avg);
    
    duty_stdev = stdev(mat_duty, duty_avg, 5);
    stdev_stdev = stdev(mat_stdev, stdev_avg, 5);
    rms_stdev = stdev(mat_rms, rms_avg, 5);

    // printf("%.4f, %.4f, %.4f\n", duty_stdev, stdev_stdev, rms_stdev);

    carpet[0] = (carpet[0] - duty_avg) / duty_stdev;
    carpet[1] = (carpet[1] - stdev_avg) / stdev_stdev;
    carpet[2] = (carpet[2] - rms_avg) / rms_stdev;

    wavy[0] = (wavy[0] - duty_avg) / duty_stdev;
    wavy[1] = (wavy[1] - stdev_avg) / stdev_stdev;
    wavy[2] = (wavy[2] - rms_avg) / rms_stdev;
    
    blacktile[0] = (blacktile[0] - duty_avg) / duty_stdev;
    blacktile[1] = (blacktile[1] - stdev_avg) / stdev_stdev;
    blacktile[2] = (blacktile[2] - rms_avg) / rms_stdev;
    
    wood[0] = (wood[0] - duty_avg) / duty_stdev;
    wood[1] = (wood[1] - stdev_avg) / stdev_stdev;
    wood[2] = (wood[2] - rms_avg) / rms_stdev;
    
    hexagon[0] = (hexagon[0] - duty_avg) / duty_stdev;
    hexagon[1] = (hexagon[1] - stdev_avg) / stdev_stdev;
    hexagon[2] = (hexagon[2] - rms_avg) / rms_stdev;


    // printf("%.4f, %.4f, %.4f, %.4f, %.4f\n", carpet[1], wavy[1], blacktile[1], wood[1], hexagon[1]);
}

float mean(float * arr, int len){
    float sum = 0;
    for(int i = 0; i < len; i++){
        sum += arr[i];
    }
    sum/=len;
    return sum;
}

float stdev(float * arr, float avg, int len){
    float stdev = 0;
    for(int i = 0; i < len; ++i){
        stdev += pow((arr[i] - avg), 2);
    }

    stdev = sqrt(stdev/len); //std devs recorded
    return stdev;
}

float sample_stdev(float * arr, float avg, int len){
    float stdev = 0;
    for(int i = 0; i < len; ++i){
        stdev += pow((arr[i] - avg), 2);
    }

    stdev = sqrt(stdev/(len-1)); //std devs recorded
    return stdev;
}



float get_dist(float * mat_data, float avg, float std_dev, float rms){
    return sqrt((mat_data[0] - avg)*(mat_data[0] - avg) + (mat_data[1] - std_dev)*(mat_data[1] - std_dev) + (mat_data[2] - rms)*(mat_data[2] - rms));
}



float get_dist_simple(float mat_avg, float avg){
    return (fabs(mat_avg - avg));
}

int majority(int *arr, int len) {
    int votes[6] = {0,0,0,0,0,0};
    
    int vote;
    
    for (int i=0; i<len; i++) {
        vote = arr[i];
        //printf("%d\n",vote);
        votes[vote]++;
    }
    
    int choice = 5;
    int max_count = 0;
    for (int i=0; i<6; ++i) {
        if (votes[i] > max_count) {
            max_count = votes[i];
            choice = i;
        }
    }

    return choice;
}
















