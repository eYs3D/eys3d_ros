#pragma once

#ifndef _SELF_CALIBRATION_API_H_
#define _SELF_CALIBRATION_API_H_

#ifdef _WIN32
#ifdef Self_Calibration_API_EXPORTS
#define LIBSELF_K_API __declspec(dllexport)
#else
#define LIBSELF_K_API __declspec(dllimport)
#endif
#endif

#ifdef __linux__
#define LIBSELF_K_API
#endif

namespace self_calibration
{
    enum self_calibration_issue
    {
        Self_Calibration_Success = 0,
        Self_Calibration_Fail = 1,
        Self_Calibration_ParameterError = 2,
        Self_Calibration_FrameError = 3,
        Self_Calibration_TareDepthError = 4,
        Self_Calibration_PlaneError = 5,
        Self_Calibration_QualityBad = 6,
        Self_Calibration_NotReady = 7,
    };

    enum calibration_mode
    {
        Tare = 0,
        Fix_intrinsic = 1,
        Fix_extrinsic = 2,
        Fix_intrinsic_extrinsic = 3,
        Fix_extrinsic_intrinsic = 4,
    };

    enum input_image_type
    {
        raw = 0,
        undistorted = 1,
        rectified = 2,
    };

    enum calibration_controller_parameters
    {
        b_do_quality_monitoring = 0,
        period_of_quality_check = 1,
        low_quality_threshold_to_trigger_warning = 2,
        quality_threshold_to_trigger_parameter_estimation_for_recovery = 3,
        b_automatically_triger_parameter_estimation_for_recovery = 4,
        target_quality_threshold_for_recovery = 5,
        recovery_mode = 6,
        accuracy_level = 7,
        n_max_recovery_trials = 8,
        tare_target = 9,
    };

    enum monitor_state
    {
        Initing,
        Idle,
        Suspended,
        Calibrating,
        CalculatingHealth,
    };

    enum parameter_state
    {
        Waiting,
        Good,
        CouldbeImproved,
        NeedCalibration,
    };

    struct module_parameters
    {
        int height = 0;
        int width = 0;
        double fixed_focus = 0;
        double left_intrinsic[9] = {0};
        double left_distortion[8] = {0};
        double left_rotation[9] = {0};
        double left_projection[12] = {0};
        double right_intrinsic[9] = {0};
        double right_distortion[8] = {0};
        double right_rotation[9] = {0};
        double right_projection[12] = {0};
        double extrinsic_rotation[9] = {0};
        double extrinsic_translation[3] = {0};
        double reprojection[16] = {0};
    };

    class LIBSELF_K_API self_calibration_api
    {
    private:
        // monitor controller
        bool b_do_quality_monitoring;
        int period_quality_check;
        float low_quality_threshold_to_trigger_warning;
        float quality_threshold_to_trigger_parameter_estimation_for_recovery;
        bool b_automatically_triger_parameter_estimation_for_recovery;
        float target_quality_threshold_for_recovery;
        float tare_distance;
        calibration_mode recovery_mode;
        int accuracy_level;
        int n_max_recovery_trials;
        input_image_type image_type;

        // flow controller
        bool flow_flag;
        monitor_state _monitor_state;
        parameter_state current_state;
        parameter_state estimated_state;

        // Parameters
        unsigned char *src_image_address;
        module_parameters default_parameters;
        module_parameters current_parameters;
        module_parameters estimated_parameters;

        // flag
        int recovery_trials;
        float score_of_current_parameters;
        float score_of_estimated_parameters;
        bool warning_flag;

        void mainflow();

    public:
        self_calibration_api();
        ~self_calibration_api();

        void initflow();
        void runflow();
        void pauseflow();
        void closeflow();

        self_calibration_issue init(module_parameters par, unsigned char *imageBuffer, calibration_mode mode, float Tare_Depth);
        self_calibration_issue init(module_parameters par, unsigned char *imageBuffer);
        self_calibration_issue init(module_parameters par, unsigned char *imageBuffer, input_image_type type);
        self_calibration_issue set_image_pointer(unsigned char *imageBuffer);
        self_calibration_issue CalculateStandardScoreForModule(double y_bias, float &score);
        self_calibration_issue doCalculatingForCurrentHealth(float &quality);
        self_calibration_issue doParameterEstimationForRecovery();
        self_calibration_issue resetCurrentAndEstimatedParametersToDefault();
        self_calibration_issue resetConfiguration();
        self_calibration_issue getMonitorState(monitor_state *state);
        self_calibration_issue getStateOfCurrentPars(parameter_state *state);
        self_calibration_issue getStateOfEstimatedPars(parameter_state *state);
        self_calibration_issue getScroeOfCurrentPars(float *score);
        self_calibration_issue getScroeOfEstimatedPars(float *score);
        self_calibration_issue setCurrentParameters(module_parameters pars);
        self_calibration_issue getCurrentParameters(module_parameters *pars);
        self_calibration_issue getEstimatedParameters(module_parameters *pars);
        self_calibration_issue getWarningFlag(bool *flag);
        self_calibration_issue updateCurrentParameters();

        self_calibration_issue set_do_quality_monitoring(bool is_monitor);
        self_calibration_issue set_period_for_quality_check(int second);
        self_calibration_issue set_low_quality_threshold_to_trigger_warning(float score);
        self_calibration_issue set_quality_threshold_to_trigger_parameter_estimation_for_recovery(float score);
        self_calibration_issue set_automatically_triger_parameter_estimation_for_recovery(bool is_auto);
        self_calibration_issue set_target_quality_threshold_for_recovery(float target);
        self_calibration_issue set_tare_distance(float distance);
        self_calibration_issue set_recovery_mode(calibration_mode mode);
        self_calibration_issue set_accuracy_level(int level);
        self_calibration_issue set_max_recovery_trials(int number);
        self_calibration_issue set_input_image_type(input_image_type type);

        self_calibration_issue get_do_quality_monitoring(bool *is_monitor);
        self_calibration_issue get_period_for_quality_check(int *second);
        self_calibration_issue get_low_quality_threshold_to_trigger_warning(float *score);
        self_calibration_issue get_quality_threshold_to_trigger_parameter_estimation_for_recovery(float *score);
        self_calibration_issue get_automatically_triger_parameter_estimation_for_recovery(bool *is_auto);
        self_calibration_issue get_target_quality_threshold_for_recovery(float *target);
        self_calibration_issue get_tare_distance(float *distance);
        self_calibration_issue get_recovery_mode(calibration_mode *mode);
        self_calibration_issue get_accuracy_level(int *level);
        self_calibration_issue get_max_recovery_trials(int *number);
        self_calibration_issue get_input_image_type(input_image_type *type);
    };

}

#endif