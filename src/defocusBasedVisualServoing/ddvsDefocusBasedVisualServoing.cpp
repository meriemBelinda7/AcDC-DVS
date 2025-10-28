#include "ddvsDefocusBasedVisualServoing.h"


ddvsDefocusBasedVisualServoing::ddvsDefocusBasedVisualServoing():Node("DefocusDirectVisualServoing")
{
this->declare_parameter<std::string>("Image_topic", "currentImage");
    cameraTopic_ = this->get_parameter("Image_topic").as_string();
    

    
    this->declare_parameter<std::string>("relative_pos_topic", "cmd/relative_pos");
     rel_pos_topic_ = this->get_parameter("relative_pos_topic").as_string();

    this->declare_parameter<std::string>("camera_velocity_topic_", "cmd/camera_velocity");
     vel_topic_ = this->get_parameter("camera_velocity_topic_").as_string();

    this->declare_parameter<std::string>("twist_topic_", "/twist_msg");
    twist_topic_ = this->get_parameter("twist_topic_").as_string(); 

    this->declare_parameter<std::string>("Log_folder", "/home/belinda-naamani/.ros/ros_dvs_bridge/");
    logs_path_ = this->get_parameter("Log_folder").as_string();

    this->declare_parameter<std::string>("diffTopic", "/Id_diff");
    diffTopic_ = this->get_parameter("diffTopic").as_string();

    this->declare_parameter<std::string>("errTopic", "/residual");
    errTopic_ = this->get_parameter("errTopic").as_string();

    this->declare_parameter<double>("Lambda", double(1.0));
    lambda_ = this->get_parameter("Lambda").as_double();

    this->declare_parameter<double>("focal", double(1.0));
    f_ = this->get_parameter("focal").as_double();

    this->declare_parameter<double>("pixelSize", double(1.0));
    ku_ = this->get_parameter("pixelSize").as_double();

    this->declare_parameter<double>("u0", double(1.0));
    u0_ = this->get_parameter("u0").as_double();    

    this->declare_parameter<double>("v0", double(1.0));
    v0_ = this->get_parameter("v0").as_double();

    this->declare_parameter<double>("sceneDepth", double(1.0));
    sceneDepth_ = this->get_parameter("sceneDepth").as_double();
    
    this->declare_parameter<bool>("controlInBaseFrame", false);
    controlInBaseFrame_ = this->get_parameter("controlInBaseFrame").as_bool();;
    
    this->declare_parameter<double>("FNumber", double(1.0));
    FNumber_ = this->get_parameter("FNumber").as_double();

    this->declare_parameter<double>("Zf", double(1.0));
    Zf_ = this->get_parameter("Zf").as_double();

    this->declare_parameter<bool>("tx", false);
    tx_ = this->get_parameter("tx").as_bool();

    this->declare_parameter<bool>("ty", false);
    ty_ = this->get_parameter("ty").as_bool();

    this->declare_parameter<bool>("tz", false);
    tz_ = this->get_parameter("tz").as_bool();

    this->declare_parameter<bool>("rx", false);
    rx_ = this->get_parameter("rx").as_bool();

    this->declare_parameter<bool>("ry", false);
    ry_ = this->get_parameter("ry").as_bool();

    this->declare_parameter<bool>("rz", false);
    rz_ = this->get_parameter("rz").as_bool();

   
    

    this->declare_parameter<double>("dtX", 0.0);
    p_init_.data.push_back(this->get_parameter("dtX").as_double());

    this->declare_parameter<double>("dtY", 0.0);
    p_init_.data.push_back(this->get_parameter("dtY").as_double());

    this->declare_parameter<double>("dtZ", 0.0);
    p_init_.data.push_back(this->get_parameter("dtZ").as_double());

    this->declare_parameter<double>("drX", 0.0);
    p_init_.data.push_back(vpMath::rad(this->get_parameter("drX").as_double()));

    this->declare_parameter<double>("drY", 0.0);
    p_init_.data.push_back(vpMath::rad(this->get_parameter("drY").as_double()));

    this->declare_parameter<double>("drZ", 0.0);
    p_init_.data.push_back(vpMath::rad(this->get_parameter("drZ").as_double()));

    this->declare_parameter<int>("binning", false);
    binning_val_ = this->get_parameter("binning").as_int();
    ku_ = binning_val_*ku_;
    if (ku_>0)
        px_ = py_ = f_/ku_;
    else 
    px_ = py_ = f_;


    iter_ = 0;
    dof_.resize(7,7);
    dof_ = 0;
}



ddvsDefocusBasedVisualServoing::~ddvsDefocusBasedVisualServoing()
{
    
    stopRobot();

}



void ddvsDefocusBasedVisualServoing::init()
{
    image_transport::ImageTransport it(shared_from_this());
    cam_.initParamsCamThinLens(f_,ku_,FNumber_,Zf_,u0_,v0_);
    
    stringstream str;
    str<<logs_path_<<"logfile.txt";
    log_file_.open(str.str().c_str());
    str.str("");
    str<<logs_path_<<"residuals.txt";
    residuals_.open(str.str().c_str());
    str.str("");
    str<<logs_path_<<"times.txt";
    times_.open(str.str().c_str());
    initVisualServoingTask();
   // L.resize(dim_s_,7);
    sub_ = it.subscribe(cameraTopic_, 1, std::bind(&ddvsDefocusBasedVisualServoing::imageCallback, this,  std::placeholders::_1));
    //velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(robotTopic_,10);
    residual_pub_ = this->create_publisher<std_msgs::msg::Float64>(errTopic_,1);
    rel_pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rel_pos_topic_, 1);
    vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(vel_topic_, 1);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(twist_topic_, 1);
    // std::cout << "going to the initial position"<<std::endl;
    // rel_pos_pub_->publish(p_init_);
    // vpTime::wait(5000);
    log_file_<<"node init"<<endl;
    // std::cout << "reached initial position"<<std::endl;
    if (diffTopic_.compare("") != 0)
    {
        diff_pub_ = it.advertise(diffTopic_, 1);
        pub_diffImage_ = true;
    }
    // p_init_.data.clear();
    twist_msg_.linear.x = 0.0;
    twist_msg_.linear.y = 0.0;
    twist_msg_.linear.z = 0.0;
    twist_msg_.angular.x = 0.0;
    twist_msg_.angular.y = 0.0;
    twist_msg_.angular.z = 0.0;
    twist_pub_->publish(twist_msg_);
   
    
}
//void ddvsDefocusBasedVisualServoing::binning(const vpImage<unsigned char>& src, vpImage<unsigned char>& dst, int binningFactor)
//{
//
//    int newWidth = src.getWidth() / binningFactor;
//    int newHeight = src.getHeight() / binningFactor;
//    dst.resize(newHeight, newWidth);
//
//    for (int i = 0; i < newHeight; ++i) {
//        for (int j = 0; j < newWidth; ++j) {
//            int sum = 0;
//            for (int bi = 0; bi < binningFactor; ++bi) {
//                for (int bj = 0; bj < binningFactor; ++bj) {
//                    sum += src[i * binningFactor + bi][j * binningFactor + bj];
//                }
//            }
//            dst[i][j] = sum / (binningFactor * binningFactor);
//        }
//    }
//}


void ddvsDefocusBasedVisualServoing::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    log_file_<<"Beginning of the visual servoing"<<endl;
    if(vsInitialized_)
    {

        iter_++;
    
            double duration = vpTime::measureTimeMs();
        
            
            current_image_ = visp_bridge::toVispImage(*msg);
            //binning(current_image_temp_, current_image_,8);
            //vpImageTools::resize(current_image_temp_, current_image_, current_image_temp_.getWidth()/8, current_image_temp_.getHeight()/8, vpImageTools::INTERPOLATION_LINEAR );
            if((current_image_.getHeight() == height_) && (current_image_.getWidth() == width_))
            {
                sI.buildFrom(current_image_);
        
                //double duration1 = vpTime::measureTimeMs();
                v_ = Servo.computeControlLaw();
                //std::cout<<"control law processing time per frame = "<< vpTime::measureTimeMs() - duration1 <<std::endl;
                
                //v_=0;
                //log_file_<<v_ << std::endl;
            
			    //robot.setCameraVelocity(v_);

    	        //velocity_pub_->publish(velocity_);

                vpColVector e = Servo.getError();
                norm_error_ = e.sumSquare();

    
                // v_msg_.data.push_back(v_[0]);
                // v_msg_.data.push_back(v_[1]);
                // v_msg_.data.push_back(v_[2]);
                // v_msg_.data.push_back(v_[3]);
                // v_msg_.data.push_back(v_[4]);
                // v_msg_.data.push_back(v_[5]);
                // vel_pub_->publish(v_msg_);

                twist_msg_.linear.x = v_[0];
                twist_msg_.linear.y = v_[1];
                twist_msg_.linear.z = v_[2];
                twist_msg_.angular.x =v_[3];
                twist_msg_.angular.y =v_[4];
                twist_msg_.angular.z =v_[5];
                
                twist_pub_->publish(twist_msg_);
                if(pub_diffImage_)
                {
      	            if(errorToImage(e, difference_image_) == 0)
      	            {
      		            diff_ = visp_bridge::toSensorMsgsImage(difference_image_);
      		            diff_pub_.publish(diff_);
      	            }
			    }

                times_ << vpTime::measureTimeMs() - duration << "ms" << std::endl;

                residuals_ << norm_error_<< std::endl;

                log_file_<< "|e| = " << norm_error_<< std::endl;
                log_file_<< "|V| = " << sqrt(v_.sumSquare())<< std::endl;
                log_file_<< "V = " << v_<< std::endl;
                //std::cout<< " v = " << v_ << std::endl;
                
            }
            else
            {
                log_file_<<"current image size " << current_image_.getWidth() << "x" << current_image_.getHeight() <<  " pixels different from the desired " <<width_ << "x" << height_<<" pixels" << endl;
                v_ = 0;


			    twist_msg_.linear.x = 0;
			    twist_msg_.linear.y = 0;
			    twist_msg_.linear.z = 0;
			    twist_msg_.angular.x = 0;
			    twist_msg_.angular.y = 0;
                twist_msg_.angular.z = 0;
			    twist_pub_->publish(twist_msg_);
            }
            residuals_msg_.data = norm_error_;
            residual_pub_->publish(residuals_msg_);
            // v_msg_.data.clear();
        }
        else 
            log_file_<<"VS not initialized, cannot control motion." << std::endl;
}



void ddvsDefocusBasedVisualServoing::initVisualServoingTask()
{
    log_file_<<"node init"<<endl;
    std::string filename_read_image;
    stringstream ss_desired_image;

    ss_desired_image<<logs_path_<<"/Id"<<".png";
    filename_read_image = vpIoTools::path(ss_desired_image.str().c_str());
    v_.resize(6);
    dof_.resize(6);
    dof_[0] = tx_;
    dof_[1] = ty_;
    dof_[2] = tz_;
    dof_[3] = rx_;
    dof_[4] = ry_;
    dof_[5] = rz_;
    std::cout<<dof_<<std::endl;
    try
    {
        
        vpImageIo::read(this-> desired_image_, filename_read_image);
        
        log_file_<<"desired_image read from"<<filename_read_image<<endl;
		height_ = desired_image_.getHeight();
		width_ = desired_image_.getWidth();
        sId.init(height_, width_,sceneDepth_);
        sId.setCameraParameters(cam_);
        sId.buildFrom(desired_image_);
        dim_s_ = sId.getDim();
        //build current features
        current_image_ = desired_image_;
        sI.init(height_, width_, sceneDepth_);
        sI.setCameraParameters(cam_);
        sI.buildFrom(current_image_);

        
        difference_image_.resize(height_, width_, true);

        Servo.setServo(vpServo::EYEINHAND_CAMERA);

	  // add current and desired visual features
	    Servo.addFeature(sI, sId);
    // set the gain
        Servo.setLambda(lambda_); 
        Servo.setCameraDoF(dof_);
    // compute interaction matrix at the desired position
        Servo.setInteractionMatrixType(vpServo::DESIRED);
        vsInitialized_ = true;

    }
    catch (const vpException &e) 
    {
        std::cout<<'desired image read successfully' <<std::endl;
        std::cout << "Catch an exception: " << e << std::endl;
        vsInitialized_ = false;
    }
    log_file_<<"Visual Servoing successfully initialized"<<endl;
}



void ddvsDefocusBasedVisualServoing::toolPoseCallback(const tf2_msgs::msg::TFMessage& tf)
{
    vpHomogeneousMatrix bMc = toVispHomogeneousMatrix(tf);
	bMc[0][3] = bMc[1][3] = bMc[2][3] = 0;
    bVc_.buildFrom(bMc);

}

vpHomogeneousMatrix ddvsDefocusBasedVisualServoing::toVispHomogeneousMatrix(const tf2_msgs::msg::TFMessage& trans)
{
    vpHomogeneousMatrix mat;
	vpTranslationVector vec(trans.transforms[0].transform.translation.x,trans.transforms[0].transform.translation.y,trans.transforms[0].transform.translation.z);
	vpRotationMatrix rmat;

	double a = trans.transforms[0].transform.rotation.w; //x
	double b = trans.transforms[0].transform.rotation.x; //y
	double c = trans.transforms[0].transform.rotation.y; //z
	double d = trans.transforms[0].transform.rotation.z; //w
	rmat[0][0] = a*a+b*b-c*c-d*d;
	rmat[0][1] = 2*b*c-2*a*d;
	rmat[0][2] = 2*a*c+2*b*d;

	rmat[1][0] = 2*a*d+2*b*c;
	rmat[1][1] = a*a-b*b+c*c-d*d;
	rmat[1][2] = 2*c*d-2*a*b;

	rmat[2][0] = 2*b*d-2*a*c;
	rmat[2][1] = 2*a*b+2*c*d;
	rmat[2][2] = a*a-b*b-c*c+d*d;

	mat.buildFrom(vec,rmat);

	return mat;
}



int ddvsDefocusBasedVisualServoing::errorToImage(vpColVector &e, vpImage<unsigned char> &diff_image)
{
    if((diff_image.getHeight() != height_) || (diff_image.getWidth() != width_))
		return -1;

	double *pt_e = e.data;	
	unsigned char *pt_Idiff;
	unsigned int bord = 10; //protected but always 10 in vpFeatureLuminance
	//for(unsigned int i = 0 ; i < e.size() ; i++, pt_Idiff++, pt_e++)
	for (unsigned int i = bord; i < height_ - bord; i++)
	{
		pt_Idiff = &(diff_image[i][bord]);
        for (unsigned int j = bord; j < width_ - bord; j++, pt_Idiff++, pt_e++)
			*pt_Idiff = (*pt_e + 255) * 0.5;
	}
	
	return 0;
}



void ddvsDefocusBasedVisualServoing::stopRobot()
{
	twist_msg_.linear.x = 0;
    twist_msg_.linear.y = 0;
    twist_msg_.linear.z = 0;
    twist_msg_.angular.x = 0;
    twist_msg_.angular.y = 0;
    twist_msg_.angular.z = 0;
    twist_pub_->publish(twist_msg_);
    // v_=0;
	//velocity_pub_->publish(velocity_);
}