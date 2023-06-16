#ifndef LABEL_RECOGNIZER_H
#define LABEL_RECOGNIZER_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <torch/torch.h>
#include <string>
#include "armor.h"

class LabelRecognizer
{
public:
    LabelRecognizer(std::string model_path); // to be modified with actual model path
    void identify(cv::Mat img, std::vector<Armor>& armors);
    std::vector<cv::Mat> preprocess(cv::Mat img, std::vector<Armor> armors);
private:
    void recognizeNumber(std::vector<cv::Mat> imgs, std::vector<Armor>& armors);
    const int processed_img_width = 20;
    const int processed_img_height = 28;
    const int warp_height = 28;
    torch::jit::script::Module model;
};

#endif