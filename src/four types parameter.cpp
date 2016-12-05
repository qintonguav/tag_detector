// fisrt AD parameter + AD function
camera_R[0] = Q.w();
camera_R[1] = Q.x();
camera_R[2] = Q.y();
camera_R[3] = Q.z();

    ceres::LocalParameterization* local_parameterization =
        new ceres::AutoDiffLocalParameterization<QuaternionPlus, 4, 3>;

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>
             (new ReprojectionError(un_pts_2[i].x, un_pts_2[i].y, pts_3[i].x, pts_3[i].y, pts_3[i].z));

             struct ReprojectionError
             {
             	ReprojectionError(double observed_u, double observed_v, double Point_x, double Point_y, double Point_z)
             		:observed_u(observed_u), observed_v(observed_v), Point_x(Point_x), Point_y(Point_y), Point_z(Point_z)
             		{}

             	template <typename T>
             	bool operator()(const T* const camera_R, const T* const camera_T, T* residuals) const
             	{
             		T p[3];
             		T point[3];
             		point[0] = T(Point_x);
             		point[1] = T(Point_y);
             		point[2] = T(Point_z);
             		ceres::QuaternionRotatePoint(camera_R, point, p);
             		p[0] += camera_T[0]; p[1] += camera_T[1]; p[2] += camera_T[2];
             		T xp = p[0] / p[2];
                 T yp = p[1] / p[2];
                 residuals[0] = xp - T(observed_u);
                 residuals[1] = yp - T(observed_v);
                 return true;
             	}

             	double observed_u;
             	double observed_v;
             	double Point_x;
             	double Point_y;
             	double Point_z;
             	/*
             	static ceres::CostFunction* Create(const double observed_u, const double observed_v, const double* Point){
             		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>(
             			new ReprojectionError(observed_u, observed_v, Point)));
             	};
             	*/
             };

             struct QuaternionPlus {
               template<typename T>
               bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
                 //std::cout << "oquaternion PLUS" << std::endl;
                 //std::cout << "delta 0 " << delta[0] << "delta 1 " << delta[1] << "delta 2 " << delta[2] << "delta 3 " << delta[3] << std::endl;
                 const T squared_norm_delta =
                     delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];

                 T q_delta[4];
                 if (squared_norm_delta > T(0.0)) {
                   T norm_delta = sqrt(squared_norm_delta);
                   const T sin_delta_by_delta = sin(norm_delta) / norm_delta;
                   q_delta[0] = cos(norm_delta);
                   q_delta[1] = sin_delta_by_delta * delta[0];
                   q_delta[2] = sin_delta_by_delta * delta[1];
                   q_delta[3] = sin_delta_by_delta * delta[2];
                 } else {
                   // We do not just use q_delta = [1,0,0,0] here because that is a
                   // constant and when used for automatic differentiation will
                   // lead to a zero derivative. Instead we take a first order
                   // approximation and evaluate it at zero.
                   q_delta[0] = T(1.0);
                   q_delta[1] = delta[0];
                   q_delta[2] = delta[1];
                   q_delta[3] = delta[2];
                 }

                 ceres::QuaternionProduct(q_delta, x, x_plus_delta);
                 return true;
               }
             };





// second manual parameter + AD function
             camera_R[0] = Q.w();
             camera_R[1] = Q.x();
             camera_R[2] = Q.y();
             camera_R[3] = Q.z();
        ceres::LocalParameterization* local_parameterization = new QuaternionParameterization();

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>
                (new ReprojectionError(un_pts_2[i].x, un_pts_2[i].y, pts_3[i].x, pts_3[i].y, pts_3[i].z)); 

            struct ReprojectionError
            {
            	ReprojectionError(double observed_u, double observed_v, double Point_x, double Point_y, double Point_z)
            		:observed_u(observed_u), observed_v(observed_v), Point_x(Point_x), Point_y(Point_y), Point_z(Point_z)
            		{}

            	template <typename T>
            	bool operator()(const T* const camera_R, const T* const camera_T, T* residuals) const
            	{
            		T p[3];
            		T point[3];
            		point[0] = T(Point_x);
            		point[1] = T(Point_y);
            		point[2] = T(Point_z);
            		ceres::QuaternionRotatePoint(camera_R, point, p);
            		p[0] += camera_T[0]; p[1] += camera_T[1]; p[2] += camera_T[2];
            		T xp = p[0] / p[2];
                T yp = p[1] / p[2];
                residuals[0] = xp - T(observed_u);
                residuals[1] = yp - T(observed_v);
                return true;
            	}

            	double observed_u;
            	double observed_v;
            	double Point_x;
            	double Point_y;
            	double Point_z;
            	/*
            	static ceres::CostFunction* Create(const double observed_u, const double observed_v, const double* Point){
            		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>(
            			new ReprojectionError(observed_u, observed_v, Point)));
            	};
            	*/
            };


            class QuaternionParameterization : public ceres::LocalParameterization
            {
                virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const
                {
                  //std::cout << "analy quaternion PLUS" << std::endl;
                  //std::cout << "delta 0 " << delta[0] << "  delta 1 " << delta[1] << "  delta 2 " << delta[2] << "  delta 3 " << delta[3] << std::endl;
                  
                  const double squared_norm_delta =
                      (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]) / 4;

                  double q_delta[4];
                  if (squared_norm_delta > 0.0) {
                    double norm_delta = sqrt(squared_norm_delta);
                    const double sin_delta_by_delta = sin(norm_delta) / norm_delta;
                    q_delta[0] = cos(norm_delta);
                    q_delta[1] = sin_delta_by_delta * delta[0];
                    q_delta[2] = sin_delta_by_delta * delta[1];
                    q_delta[3] = sin_delta_by_delta * delta[2];
                  } else {
                    // We do not just use q_delta = [1,0,0,0] here because that is a
                    // constant and when used for automatic differentiation will
                    // lead to a zero derivative. Instead we take a first order
                    // approximation and evaluate it at zero.
                    q_delta[0] = double(1.0);
                    q_delta[1] = delta[0];
                    q_delta[2] = delta[1];
                    q_delta[3] = delta[2];
                  }

                  ceres::QuaternionProduct(q_delta, x, x_plus_delta);
                  //ceres::QuaternionProduct(x, q_delta, x_plus_delta);
                  return true;
                  
                }

                 virtual bool ComputeJacobian(const double *x, double *jacobian) const
                {
                  
                    //std::cout << "analy quaternion ComputeJacobian" << std::endl;

                  
                    
                    jacobian[0] = -x[1]; jacobian[1]  = -x[2]; jacobian[2]  = -x[3];  // NOLINT
                    jacobian[3] =  x[0]; jacobian[4]  =  x[3]; jacobian[5]  = -x[2];  // NOLINT
                    jacobian[6] = -x[3]; jacobian[7]  =  x[0]; jacobian[8]  =  x[1];  // NOLINT
                    jacobian[9] =  x[2]; jacobian[10] = -x[1]; jacobian[11] =  x[0];  // NOLINT
                    return true;
                }
                virtual int GlobalSize() const { return 4; };
                virtual int LocalSize() const { return 3; };
            };

//third manual + manual
camera_R[0] = Q.x();
camera_R[1] = Q.y();
camera_R[2] = Q.z();
camera_R[3] = Q.w();

ceres::LocalParameterization* local_parameterization = new QuaternionParameterization();

ceres::CostFunction* cost_function  = 
    new ReprojectionCostFunction(un_pts_2[i].x, un_pts_2[i].y, pts_3[i].x, pts_3[i].y, pts_3[i].z);

    class ReprojectionCostFunction : public ceres::SizedCostFunction<2, 4, 3>
    {
        public:
            ReprojectionCostFunction(double observed_u, double observed_v, double Point_x, double Point_y, double Point_z)
            :observed_u(observed_u), observed_v(observed_v), Point_x(Point_x), Point_y(Point_y), Point_z(Point_z)
            {}
            virtual ~ ReprojectionCostFunction() {}
            virtual bool Evaluate(double const* const* parameters, double* residuals,
                              double** jacobians) const
        {
            //double point[3], p[3];
            //point[0] = Point_x;
            //point[1] = Point_y;
            //point[2] = Point_z;
            Vector3d Point(Point_x, Point_y, Point_z);
            Eigen::Map<const Eigen::Vector3d> T(parameters[1]);
            Eigen::Map<const Eigen::Quaterniond> Q(parameters[0]);
            Vector3d Re_point;
            Re_point = Q * Point + T;
            //ceres::QuaternionRotatePoint(parameters[0], point, p);
            //p[0] += parameters[1][0]; p[1] += parameters[1][1]; p[2] += parameters[1][2];
            residuals[0] = Re_point(0) / Re_point(2) - observed_u;
            residuals[1] = Re_point(1) / Re_point(2) - observed_v;
            MatrixXd H(2, 3);
            H << 1.0 / Re_point(2), 0,               -Re_point(0) / (Re_point(2) * Re_point(2)),
                 0 ,              1.0 / Re_point(2), -Re_point(1) / (Re_point(2) * Re_point(2));
            //Eigen::Quaterniond Q(parameters[0][0], parameters[0][1], parameters[0][2], parameters[0][3]);
            Matrix3d R;
            R = Q.toRotationMatrix();

            if (jacobians != NULL && jacobians[0] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_Q(jacobians[0]);
                jacobian_Q.setZero();
                jacobian_Q.rightCols<3>() = H * R * -skewSymmetric(Point);
            }
            if (jacobians != NULL && jacobians[1] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_T(jacobians[1]);
                jacobian_T = H;
            }
            return true;
        }

        double observed_u;
        double observed_v;
        double Point_x;
        double Point_y;
        double Point_z;
    };


    class QuaternionParameterization : public ceres::LocalParameterization
    {
        virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const
        {

          Eigen::Map<const Eigen::Quaterniond> _q(x);
          Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Vector3d(delta[0], delta[1], delta[2]));
          //Eigen::Quaterniond q;
          Eigen::Map<Eigen::Quaterniond> q(x_plus_delta);
          q = (_q * dq).normalized();
          //cout << " q_delta my " << dq.normalize().w() <<"  " <<  dq.normalize().x() <<"  " <<  dq.normalize().y() <<"  " <<  dq.normalize().z() << endl;
          
          return true;
        }

         virtual bool ComputeJacobian(const double *x, double *jacobian) const
        {

            Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
            j.topRows<1>().setZero();
            j.bottomRows<3>().setIdentity();
            return true;
        }
        virtual int GlobalSize() const { return 4; };
        virtual int LocalSize() const { return 3; };
    };

//third manual(ceres_extensions.h) + AD

    camera_R[0] = Q.w();
    camera_R[1] = Q.x();
    camera_R[2] = Q.y();
    camera_R[3] = Q.z();


    ceres::LocalParameterization* local_parameterization = new ceres_ext::EigenQuaternionParameterization();

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>
            (new ReprojectionError(un_pts_2[i].x, un_pts_2[i].y, pts_3[i].x, pts_3[i].y, pts_3[i].z));


            struct ReprojectionError
            {
                ReprojectionError(double observed_u, double observed_v, double Point_x, double Point_y, double Point_z)
                    :observed_u(observed_u), observed_v(observed_v), Point_x(Point_x), Point_y(Point_y), Point_z(Point_z)
                    {}

                template <typename T>
                bool operator()(const T* const camera_R, const T* const camera_T, T* residuals) const
                {
                    T p[3];
                    T point[3];
                    point[0] = T(Point_x);
                    point[1] = T(Point_y);
                    point[2] = T(Point_z);
                    ceres::QuaternionRotatePoint(camera_R, point, p);
                    p[0] += camera_T[0]; p[1] += camera_T[1]; p[2] += camera_T[2];
                    T xp = p[0] / p[2];
                T yp = p[1] / p[2];
                residuals[0] = xp - T(observed_u);
                residuals[1] = yp - T(observed_v);
                return true;
                }

                double observed_u;
                double observed_v;
                double Point_x;
                double Point_y;
                double Point_z;
                /*
                static ceres::CostFunction* Create(const double observed_u, const double observed_v, const double* Point){
                    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>(
                        new ReprojectionError(observed_u, observed_v, Point)));
                };
                */
            };