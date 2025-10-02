#include "misora2_cracks/detection.hpp"

int main(int argc, char *argv[]) {
    // std::string image_file;

    // if(argc > 1)image_file = argv[1];
    // else {
    //     std::cout << "Please input image path" << std::endl;
    //     return 0;
    // }
    // // 初期設定-----------------------------------------------------
    // if (!std::filesystem::exists(Detection::MODEL_PATH)) {
    //     std::cerr << "Model file does not exist at path: " << Detection::MODEL_PATH << std::endl;
    //     throw std::runtime_error("Model file not found.");
    // }
    // // モデルの準備
    // AutoBackendOnnx model(Detection::MODEL_PATH, Detection::ONNX_LOGID, Detection::ONNX_PROVIDER);
    // // std::cout << image_file << 1 << std::endl;
    // std::vector<cv::Scalar> colors = Detection::generateRandomColors(model.getNc(), model.getCh());
    // std::unordered_map<int, std::string> names = model.getNames();
    // // 画像の読み込み
    // cv::Mat img = cv::imread(image_file, cv::IMREAD_UNCHANGED);
    // // 推論実行
    // std::vector<YoloResults> objs = model.predict_once(
    //     img,
    //     Detection::CONF_THRESHOLD,
    //     Detection::IOU_THRESHOLD,
    //     Detection::MASK_THRESHOLD,
    //     Detection::CONVERSION_CODE
    // );
    // // -------------------------------------------------------------------------------------
    // if (img.empty()) {
    //     std::cerr << "画像の読み込みに失敗しました: " << image_file << std::endl;
    //     return -1;
    // }

    // // std::cout << "Processing image: " << image_file << " (size: " << img.size() << ")" << std::endl;
    // // // 結果の描画
    // // cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    // // std::cout << "Num of objects :" << size(objs) << std::endl;
    // // auto [trimmed, boxed] = Detection::plot_results(img, objs, colors, names);
    // // if(trimmed.channels() == 1) std::cout << "Not found" << std::endl;
    // // else 
    // // {
    // //     std::cout << "trimmed: " << trimmed.size() << std::endl;
    // //     cv::imshow("with box",boxed);
    // //     cv::imshow("trimmed",trimmed);
    // //     cv::waitKey(0);
    // //     cv::destroyAllWindows();
    // // }

    
    // ディレクトリごとに処理
    std::string folder_path = "../20250922_cracks/";
    std::string image_file;
    // モデルの初期化
    AutoBackendOnnx model(Detection::MODEL_PATH, Detection::ONNX_LOGID, Detection::ONNX_PROVIDER);
    // std::cout << image_file << 1 << std::endl;
    std::vector<cv::Scalar> colors = Detection::generateRandomColors(model.getNc(), model.getCh());
    std::unordered_map<int, std::string> names = model.getNames();

    // 処理に掛けるファイルの総数
    int max_num = 75;

    for( int i = 60 ; i <= max_num ; i++){
        image_file = folder_path + "cracks_result_" + std::to_string(i) + ".png";
        std::cout << image_file << std::endl;
        
        // 画像の読み込み
        cv::Mat img = cv::imread(image_file, cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            std::cerr << "画像の読み込みに失敗しました: " << image_file << std::endl;
            continue;
        }
        // 推論実行
        std::vector<YoloResults> objs = model.predict_once(
            img,
            Detection::CONF_THRESHOLD,
            Detection::IOU_THRESHOLD,
            Detection::MASK_THRESHOLD,
            Detection::CONVERSION_CODE
        );
        // -------------------------------------------------------------------------------------
        
        if (img.empty()) {
            std::cerr << "画像の読み込みに失敗しました: " << image_file << std::endl;
            return -1;
        }
        if(objs.empty()){
            std::cout << "Not Found" << std::endl;
        }
        std::cout << size(objs) <<std::endl;
        std::cout << "Processing image: " << image_file << " (size: " << img.size() << ")" << std::endl;
        // 結果の描画
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
        auto [trimmed, boxed] = Detection::plot_results(img, objs, colors, names);
        if(trimmed.channels() == 1){
            std::cout << "Not found" << std::endl;
        }
        else {
            std::cout << "trimmed: " << trimmed.size() << std::endl;
            // cv::imshow("trimmed",trimmed);
            // cv::imshow("with box",boxed);
            // cv::waitKey(0);
            // cv::destroyAllWindows();

            std::string save_file_cropped = "../20250922_cracks/cropped/img" + std::to_string(i) + ".png";

            std::string save_file_detected = "../20250922_cracks/detected/img" + std::to_string(i) + ".png";
            cv::imwrite(save_file_cropped, trimmed);
            cv::imwrite(save_file_detected,boxed);
        }
    } 


    return 0;
}
