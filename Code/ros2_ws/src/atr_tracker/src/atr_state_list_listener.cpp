#include <memory>
#include <fstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"

#include "atr_interfaces/msg/atr_state_list_stamped.hpp"
#include "atr_interfaces/msg/atr_time_stamps.hpp"
#include "atr_interfaces/msg/atr_time_stamps_array.hpp"
#include "atr_interfaces/srv/start_measurement_srv.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ATRStateListListener : public rclcpp::Node
{
public:
    ATRStateListListener()
    : Node("atr_state_list_listener")
    {
        this->declare_parameter<std::string>("log_file","measurement");
        this->get_parameter("log_file",log_file_name);
        log_file_name = "/home/ljdust/Thesis_DPAC/thesis-work/code/src/measurement_results/"+log_file_name+".csv";

        subscription_ = this->create_subscription<atr_interfaces::msg::ATRStateListStamped>(
        "atr_state_list", 10, std::bind(&ATRStateListListener::topic_callback, this, _1));

        service_ = this->create_service<atr_interfaces::srv::StartMeasurementSrv>("/Start_Measurement", std::bind(&ATRStateListListener::measurement_callback,this,_1,_2));


    }

private:
    void measurement_callback(const std::shared_ptr<atr_interfaces::srv::StartMeasurementSrv::Request> request,
    std::shared_ptr<atr_interfaces::srv::StartMeasurementSrv::Response> response){
        
        RCLCPP_INFO(this->get_logger(), "START MEASUREMENT");
        max_samples = request->window;
        measurement = true;
        response->ack = 1;

    }

    void topic_callback(const atr_interfaces::msg::ATRStateListStamped::SharedPtr msg)
    {
        
        rclcpp::Time rec_time = now();
        
        if(measurement){
            atr_interfaces::msg::ATRTimeStampsArray times;
            atr_interfaces::msg::ATRTimeStamps time_robot;

            for(std::int64_t i=0; i<std::int64_t(msg->list.time_stamps.size());i++){
                time_robot.id = msg->list.time_stamps[i].id;
                time_robot.stamp_creation = msg->list.time_stamps[i].stamp_creation;
                time_robot.stamp_reception_tracker = msg->list.time_stamps[i].stamp_reception;
                time_robot.stamp_sending_tracker = msg->header.stamp;
                time_robot.stamp_reception_list = rec_time;
                times.iterations.push_back(time_robot);
            }
            //results.push_back(NULL);
            results.push_back(times);
            samples++;
            if(samples >= max_samples){
                measurement_idx++;
                
                std::ofstream logfile;
                logfile.open(log_file_name,std::ios_base::app);

                for(std::int64_t i = 0; i<std::int64_t(results.size());i++){
                    for(int m = 0; m<4; m++){
                        for(std::int64_t n = 0; n<std::int64_t(results[i].iterations.size());n++){
                            if(n !=0){
                                logfile<<";";
                            }
                            switch (m){
                                case 0:
                                    logfile<<results[i].iterations[n].stamp_creation.sec%1000;
                                    logfile<<".";
                                    logfile<< std::fixed << std::setprecision(9)<< std::setfill('0')<<std::setw(9)<<results[i].iterations[n].stamp_creation.nanosec;
                                break;
                                case 1:
                                    logfile<<results[i].iterations[n].stamp_reception_tracker.sec%1000;
                                    logfile<<".";
                                    logfile<< std::fixed << std::setprecision(9)<< std::setfill('0')<<std::setw(9)<<results[i].iterations[n].stamp_reception_tracker.nanosec;
                                break;
                                case 2:
                                    logfile<<results[i].iterations[n].stamp_sending_tracker.sec%1000;
                                    logfile<<".";
                                    logfile<< std::fixed << std::setprecision(9)<< std::setfill('0')<<std::setw(9)<<results[i].iterations[n].stamp_sending_tracker.nanosec;
                                break;
                                case 3:
                                    logfile<<results[i].iterations[n].stamp_reception_list.sec%1000;
                                    logfile<<".";
                                    logfile<< std::fixed << std::setprecision(9)<< std::setfill('0')<<std::setw(9)<<results[i].iterations[n].stamp_reception_list.nanosec;
                                break;
                                default:

                                break;
                            }
                        }
                        logfile <<"\n";
                    }
                }
                
                logfile.close();
                results.clear();
                measurement = false;
                samples = 0;
                RCLCPP_INFO(this->get_logger(), "MEASUREMENT FINISHED");
                
            }
        }

        auto a = msg;
    }
    
    bool measurement = false;
    std::int32_t samples = 0;
    std::int32_t max_samples= 0;
    std::int32_t  measurement_idx=0;
    std::string log_file_name;
    std::vector<atr_interfaces::msg::ATRTimeStampsArray> results;
    rclcpp::Subscription<atr_interfaces::msg::ATRStateListStamped>::SharedPtr subscription_;
    rclcpp::Service<atr_interfaces::srv::StartMeasurementSrv>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ATRStateListListener>());
    rclcpp::shutdown();
    return 0;
}