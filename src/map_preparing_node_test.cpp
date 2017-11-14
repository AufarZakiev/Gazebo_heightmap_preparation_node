#include "ros/ros.h"
#include <Magick++.h>
#include <map>
#include <boost/program_options.hpp>

using namespace Magick;
using namespace std;
namespace po = boost::program_options;

#define USAGE "See docs for usage https://github.com/AufarZakiev/Gazebo_heightmap_preparation_node"\


bool is_point_exist(size_t row, size_t column, size_t width, size_t height) {
	if ((row >= height) || (column >= width) || (row == SIZE_MAX) || (column == SIZE_MAX) || (row == SIZE_MAX - 1) || (column == SIZE_MAX - 1) ) return false;
	return true;
}

int convert_to_index(size_t i, size_t column, size_t width) {
	return (width * i) + column;
}

Color median_color_value(size_t i, size_t j, Image *input_image) {
	size_t w = input_image->columns();
	size_t h = input_image->rows();
	PixelPacket *pixels = input_image->getPixels(0, 0, w, h);
	std::map<Color, int> neighbours;
	neighbours.clear();
	neighbours[pixels[convert_to_index(i, j, w)]]++;
	if (is_point_exist(i - 1, j - 1, w, h)) {
		neighbours[pixels[convert_to_index(i - 1, j - 1, w)]]++;
	}
	if (is_point_exist(i - 1, j, w, h)) {
		neighbours[pixels[convert_to_index(i - 1, j, w)]]++;
	}
	if (is_point_exist(i - 1, j + 1, w, h)) {
		neighbours[pixels[convert_to_index(i - 1, j + 1, w)]]++;
	}
	if (is_point_exist(i, j - 1, w, h)) {
		neighbours[pixels[convert_to_index(i, j - 1, w)]]++;
	}
	if (is_point_exist(i, j + 1, w, h)) {
		neighbours[pixels[convert_to_index(i, j + 1, w)]]++;
	}
	if (is_point_exist(i + 1, j - 1, w, h)) {
		neighbours[pixels[convert_to_index(i + 1, j - 1, w)]]++;
	}
	if (is_point_exist(i + 1, j, w, h)) {
		neighbours[pixels[convert_to_index(i + 1, j, w)]]++;
	}
	if (is_point_exist(i + 1, j + 1, w, h)) {
		neighbours[pixels[convert_to_index(i + 1, j + 1, w)]]++;
	}
	Color max_value = neighbours.find(pixels[convert_to_index(i, j, w)])->first;
	int max_value_count = 0;
	for (std::map<Color, int>::iterator it = neighbours.begin(); it != neighbours.end(); ++it) {
		if (it->second > max_value_count) {
			max_value_count = it->second;
			max_value = it->first;
		}
	}
	return max_value;
}


void modify_median_color_value(size_t i, size_t j, Image *smoothed_image, Image *filtered_image) {
	size_t w = filtered_image->columns();
	size_t h = filtered_image->rows();
	PixelPacket *smoothed_pixels = smoothed_image->getPixels(0, 0, w, h);
	PixelPacket *filtered_pixels = filtered_image->getPixels(0, 0, w, h);
	if (is_point_exist(i - 1, j, w, h) && is_point_exist(i + 1, j, w, h)) {
		if (smoothed_pixels[convert_to_index(i - 1, j, w)] == smoothed_pixels[convert_to_index(i, j, w)] &&
		        smoothed_pixels[convert_to_index(i, j, w)] == smoothed_pixels[convert_to_index(i + 1, j, w)]) {
			filtered_pixels[convert_to_index(i - 1, j, w)] = smoothed_pixels[convert_to_index(i - 1, j, w)];
			filtered_pixels[convert_to_index(i, j, w)] = smoothed_pixels[convert_to_index(i, j, w)];
			filtered_pixels[convert_to_index(i + 1, j, w)] = smoothed_pixels[convert_to_index(i + 1, j, w)];
		}
	}
	if (is_point_exist(i, j - 1, w, h) && is_point_exist(i, j + 1, w, h)) {
		if (smoothed_pixels[convert_to_index(i, j - 1, w)] == smoothed_pixels[convert_to_index(i, j, w)] &&
		        smoothed_pixels[convert_to_index(i, j, w)] == smoothed_pixels[convert_to_index(i, j + 1, w)]) {
			filtered_pixels[convert_to_index(i, j - 1, w)] = smoothed_pixels[convert_to_index(i, j - 1, w)];
			filtered_pixels[convert_to_index(i, j, w)] = smoothed_pixels[convert_to_index(i, j, w)];
			filtered_pixels[convert_to_index(i, j + 1, w)] = smoothed_pixels[convert_to_index(i, j + 1, w)];
		}
	}
}

Image smooth_image(Image *input_image, size_t bot_trshd, size_t top_trshd) {
	size_t w = input_image->columns();
	size_t h = input_image->rows();
	Image smoothed_image(*input_image);
	unsigned char *gray_pixels = new unsigned char[w * h];
	smoothed_image.write(0, 0, w, h, "I", CharPixel, gray_pixels);
	ROS_INFO("top_trshd: %zu", top_trshd);
	ROS_INFO("bot_trshd: %zu", bot_trshd);
	for (size_t i = 0; i < h; i++) {
		for (size_t j = 0; j < w; j++) {
			if (gray_pixels[convert_to_index(i, j, w)] < bot_trshd)
			{
				gray_pixels[convert_to_index(i, j, w)] = 0;
			} else if (gray_pixels[convert_to_index(i, j, w)] >= top_trshd) {
				gray_pixels[convert_to_index(i, j, w)] = 255;
			}
		}
	}
	smoothed_image.read(w, h, "I", CharPixel, gray_pixels);
	return smoothed_image;
}

Image median_filter_image(Image *input_image, size_t filter_iterations) {
	size_t w = input_image->columns();
	size_t h = input_image->rows();
	PixelPacket *pixels = input_image->getPixels(0, 0, w, h);
	Image filtered_image(*input_image);
	PixelPacket *filtered_pixels = filtered_image.getPixels(0, 0, w, h);
	for (size_t count = 0; count < filter_iterations; count++) {
		for (size_t i = 0; i < h; i++) {
			for (size_t j = 0; j < w; j++) {
				filtered_pixels[convert_to_index(i, j, w)] = median_color_value(i, j, input_image);
			}
		}
		pixels = filtered_pixels;
	}
	return filtered_image;
}

Image modified_median_filter_image(Image *input_image, size_t filter_iterations) {
	size_t w = input_image->columns();
	size_t h = input_image->rows();
	PixelPacket *pixels = input_image->getPixels(0, 0, w, h);
	Image filtered_image(*input_image);
	PixelPacket *filtered_pixels = filtered_image.getPixels(0, 0, w, h);
	for (size_t count = 0; count < filter_iterations; count++) {
		for (size_t i = 0; i < h; i++) {
			for (size_t j = 0; j < w; j++) {
				filtered_pixels[convert_to_index(i, j, w)] = median_color_value(i, j, input_image);
			}
		}
		for (size_t i = 0; i < h; i++) {
			for (size_t j = 0; j < w; j++) {
				modify_median_color_value(i, j, input_image, &filtered_image);
			}
		}
		pixels = filtered_pixels;
	}
	return filtered_image;
}

void display_parameters(po::variables_map vm) {
	if (vm.count("s")) {
		cout << "Saving path was set to "
		     << vm["s"].as<std::string>() << "\n";
	} else {
		cout << "Saving path was not set. Using source image folder as default. \n";
	}

	if (vm.count("offset")) {
		std::vector<size_t> off = vm["offset"].as<std::vector<size_t> >();
		cout << "Offset_x was set to "
		     << off[0] << "\n";
		cout << "Offset_y was set to "
		     << off[1] << "\n";
	} else {
		cout << "Offset was not set. Using default 0,0 values. \n";
	}

	if (vm.count("w")) {
		cout << "Desired width was set to "
		     << vm["w"].as<size_t>() << "\n";
	} else {
		cout << "Desired width was not set. \n";
	}

	if (vm.count("h")) {
		cout << "Desired height was set to "
		     << vm["h"].as<size_t>() << "\n";
	} else {
		cout << "Desired height was not set. \n";
	}

	if (vm.count("use_median_filtering")) {
		cout << "Use_median_filtering was set to "
		     << vm["use_median_filtering"].as<bool>() << "\n";
	}

	if (vm.count("use_modified_median_filtering")) {
		cout << "Use_modified_median_filtering was set to "
		     << vm["use_modified_median_filtering"].as<bool>() << "\n";
	}

	if (vm.count("color_inverse")) {
		cout << "Color_inverse was set to "
		     << vm["color_inverse"].as<bool>() << "\n";
	}

	if (vm.count("bot_trshd")) {
		cout << "Bot_trshd was set to "
		     << vm["bot_trshd"].as<size_t>() << "\n";
	} else {
		cout << "Bot_trshd was not set. \n";
	}

	if (vm.count("top_trshd")) {
		cout << "Top_trshd was set to "
		     << vm["top_trshd"].as<size_t>() << "\n";
	} else {
		cout << "Top_trshd was not set. \n";
	}

	if (vm.count("iter")) {
		cout << "Filter_iterations was set to "
		     << vm["iter"].as<size_t>() << "\n";
	} else {
		cout << "Filter_iterations was not set. \n";
	}
}

bool check_parameters(po::variables_map vm) {
	if (vm.count("help")) {
		return false;
	}
	if (vm.count("f")) {
		cout << "Source image path was set to "
		     << vm["f"].as<std::string>() << "\n";
	} else {
		return false;
	}
	return true;
}

size_t gazebo_specified_size(size_t desired_size) {
	size_t calculated_size = 1;
	while ((calculated_size + 1) < desired_size) {
		calculated_size = calculated_size * 2;
	}
	calculated_size += 1; // Gazebo needs (2^n)+1 px size images
	return calculated_size;
}
int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "map_preparing_test");
	puts(USAGE);

	std::string map_path = "";
	std::string save_path = "";
	bool use_median_filtering;
	bool use_modified_median_filtering;
	bool color_inverse;
	size_t bot_trshd;
	size_t top_trshd;
	size_t filter_iterations;

	size_t found = map_path.find_last_of("/");
	save_path = map_path.substr(0, found);

	size_t min_width;
	size_t min_height;
	std::vector<size_t> offset_pair(0, 0);

	Image source_image;

	po::options_description desc("Allowed options");
	desc.add_options()
	("help", "Show help message")
	("f", po::value<std::string>(&map_path), "Path to source image")
	("s", po::value<std::string>(&save_path)->default_value(map_path.substr(0, map_path.find_last_of("/"))), "Prepared map and generated world saving folder path")
	("offset", po::value<std::vector<size_t> >(&offset_pair)->multitoken(), "Offset to crop from left-top corner of image")
	("w", po::value<size_t>(&min_width)->default_value(source_image.baseColumns()), "Minimum desired width. Desired width is used to compute final size of image: Gazebo hrighmap needs 2^n + 1 pixel size images, so desired size increases to nearest 2^n + 1 size. For example, 400 px turns to 513 px size.")
	("h", po::value<size_t>(&min_height)->default_value(source_image.baseRows()), "Minimum desired height (optional). Desired height is used to compute final size of image: Gazebo hrighmap needs 2^n + 1 pixel size images, so desired size increases to nearest 2^n + 1 size. For example, 400 px turns to 513 px size.")
	("use_median_filtering", po::bool_switch(&use_median_filtering)->default_value(false), "Median filter enabling")
	("use_modified_median_filtering", po::bool_switch(&use_modified_median_filtering)->default_value(false), "Modified median filter enabling")
	("color_inverse", po::bool_switch(&color_inverse)->default_value(false), "Image color inverse")
	("bot_trshd", po::value<size_t>(&bot_trshd)->default_value(0), "Thresholds used to smooth out heightmap. Every pixel below bot_trshd turns to 0 value")
	("top_trshd", po::value<size_t>(&top_trshd)->default_value(255), "Thresholds used to smooth out heightmap. Every pixel above top_trshd turns to 255 value")
	("iter", po::value<size_t>(&filter_iterations), "Filtering iterations count");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);
	if (!check_parameters(vm)) {
		cout << desc << "\n";
		return 1;
	}
	display_parameters(vm);
	source_image = Image(map_path);

	size_t max_size = 0; //TODO с заполением пикселей при отсутствии обрезки
	if (min_width != 0) {
		max_size = min_width;
	}
	if (min_height > min_width) {
		max_size = min_height;
	}

	Image sub_image(source_image);
	size_t calculated_size = gazebo_specified_size(max_size);

	sub_image.magick("png");
	sub_image.chop(Geometry(offset_pair[0], offset_pair[1]));
	sub_image.extent(Geometry(calculated_size, calculated_size), sub_image.getPixels(0, 0, sub_image.columns(), sub_image.rows())[0], NorthWestGravity);
	sub_image.type(GrayscaleType);

	Image smoothed_image = smooth_image(&sub_image, bot_trshd, top_trshd);

	Image filtered_image(smoothed_image);
	if (use_modified_median_filtering) {
		filtered_image = modified_median_filter_image(&smoothed_image, filter_iterations);
	} else {
		if (use_median_filtering) {
			filtered_image = median_filter_image(&smoothed_image, filter_iterations);
		}
	}
	if (color_inverse) {
		filtered_image.negate();
	}
	
	save_path += "/prepared_map.png";
	filtered_image.write(save_path);
	return 0;
}