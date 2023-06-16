#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <torch/script.h>
#include <string>
#include "label_recognizer.h"
#include "armor.h"
#include <iostream>

LabelRecognizer::LabelRecognizer(std::string model_path)
{
    /*
    try
    {
        model = torch::jit::load(model_path);
    }
    catch(const c10::Error& e)
    {
        std::cerr << "Error loading model: " << e.what() << std::endl;
    }
    */
}

void LabelRecognizer::identify(cv::Mat img, std::vector<Armor>& armors)
{
    std::vector<cv::Mat> processed_imgs = preprocess(img, armors);
    recognizeNumber(processed_imgs, armors);
}

std::vector<cv::Mat> LabelRecognizer::preprocess(cv::Mat img, std::vector<Armor> armors)
{
    std::vector<cv::Mat> result;
    for (Armor armor: armors)
    {
        int warp_width = 24;// (armor.label == 1 || armor.label == 7) ? 54 : 32; // to be changed with actual size
        cv::Point2f source[4] = {armor.upper_l, armor.lower_l, armor.lower_r, armor.upper_r};
        cv::Point2f target[4] = {cv::Point(0, 0), cv::Point(0, warp_height - 1), 
            cv::Point(warp_width - 1, warp_height - 1), cv::Point(warp_width - 1, 0)};
        cv::Mat mtx = cv::getPerspectiveTransform(source, target);
        cv::Mat number_img;
        cv::warpPerspective(img, number_img, mtx, cv::Size(warp_width, warp_height));
        number_img = number_img(cv::Rect(cv::Point((warp_width - processed_img_width) / 2, 0), 
            cv::Size(processed_img_width, processed_img_height)));
        //std::cout << number_img.size() << std::endl;
        cv::cvtColor(number_img, number_img, cv::COLOR_RGB2GRAY);
        cv::threshold(number_img, number_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        result.push_back(number_img);
    }
    return result;
}

void LabelRecognizer::recognizeNumber(std::vector<cv::Mat> imgs, std::vector<Armor>& armors)
{
    // TBD
}
