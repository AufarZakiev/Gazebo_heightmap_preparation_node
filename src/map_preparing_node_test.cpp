#include "ros/ros.h"
#include <Magick++.h>
#include <map>

using namespace Magick;

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

int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "map_preparing_test");
	std::string map_path = "";
	std::string save_path = "";
	size_t x_offset = 0;
	size_t y_offset = 0;
	puts(USAGE);
	if (!strcmp(argv[1], "-f"))
	{
		if (2 < argc)
			map_path = argv[2];
		else
		{
			puts(USAGE);
			return 1;
		}
	} else {
		puts(USAGE);
		return 1;
	}

	size_t found = map_path.find_last_of("/");
	save_path = map_path.substr(0, found);

	Image source_image(map_path);
	bool use_median_filtering = false;
	bool use_modified_median_filtering = false;
	bool color_inverse = false;
	size_t temp = 0;
	size_t bot_trshd = 205;
	size_t top_trshd = 205;
	size_t min_width = source_image.baseColumns();
	size_t min_height = source_image.baseRows();
	size_t filter_iterations = 1;
	size_t max_size = 0; //TODO с заполением пикселей при отсутствии обрезки

	for (int i = 3; i < argc; i++)
	{
		if (!strcmp(argv[i], "-s"))
		{
			if (i + 1 < argc) {
				i++;
				save_path = argv[i];
			}
			else
			{
				puts(USAGE);
				return 1;
			}
		}
		else if (!strcmp(argv[i], "-offset"))
		{
			if (i + 1 < argc) {
				i++;
				temp = atoi(argv[i]);
				if (temp <= source_image.baseColumns()) {
					x_offset = temp;
				}
				++i;
				temp = atoi(argv[i]);
				if (temp <= source_image.baseRows()) {
					y_offset = temp;
				}
			}
			else
			{
				puts(USAGE);
				return 1;
			}
		}
		else if (!strcmp(argv[i], "-w"))
		{
			if (i + 1 < argc) {
				i++;
				min_width = atoi(argv[i]);
			}
		}
		else if (!strcmp(argv[i], "-h"))
		{
			if (i + 1 < argc) {
				i++;
				min_height = atoi(argv[i]);
			}
		}
		else if (!strcmp(argv[i], "-use_median_filtering"))
		{
			if (i + 1 < argc) {
				i++;
				if (!strcmp(argv[i], "true")) {
					use_median_filtering = true;
				} else {
					use_median_filtering = false;
				}
			}
		}
		else if (!strcmp(argv[i], "-use_modified_median_filtering"))
		{
			if (i + 1 < argc) {
				i++;
				if (!strcmp(argv[i], "true")) {
					use_modified_median_filtering = true;
				} else {
					use_modified_median_filtering = false;
				}
			}
		}
		else if (!strcmp(argv[i], "-color_inverse"))
		{
			if (i + 1 < argc) {
				i++;
				if (!strcmp(argv[i], "true")) {
					color_inverse = true;
				} else {
					color_inverse = false;
				}
			}
		}
		else if (!strcmp(argv[i], "-top_trshd"))
		{
			if (i + 1 < argc) {
				i++;
				top_trshd = atoi(argv[i]);
			}
		}
		else if (!strcmp(argv[i], "-bot_trshd"))
		{
			if (i + 1 < argc) {
				i++;
				bot_trshd = atoi(argv[i]);
			}
		}
		else if (!strcmp(argv[i], "-iter"))
		{
			if (i + 1 < argc) {
				i++;
				filter_iterations = atoi(argv[i]);
			}
		}
		ROS_INFO("args: %s", argv[i]);
	}


	ROS_INFO("Bools: %d %d", use_median_filtering, color_inverse);
	if (min_width != 0) {
		max_size = min_width;
	}
	if (min_height > min_width) {
		max_size = min_height;
	}

	Image sub_image(source_image);
	ROS_INFO("Min desired size: %zu", max_size);
	size_t calculated_size = 1;
	while ((calculated_size + 1) < max_size) {
		calculated_size = calculated_size * 2;
	}
	calculated_size += 1; // Gazebo needs (2^n)+1 px size images

	ROS_INFO("Calculated size: %zu", calculated_size);

	sub_image.magick("png");
	sub_image.chop(Geometry(x_offset, y_offset));
	sub_image.extent(Geometry(calculated_size, calculated_size), sub_image.getPixels(0, 0, sub_image.columns(), sub_image.rows())[0], NorthWestGravity);
	sub_image.type(GrayscaleType);

	Image smoothed_image = smooth_image(&sub_image, bot_trshd, top_trshd);

	Image filtered_image(smoothed_image);
	if (use_modified_median_filtering == true) {
		filtered_image = modified_median_filter_image(&smoothed_image, filter_iterations);
	} else {
		if (use_median_filtering == true) {
			filtered_image = median_filter_image(&smoothed_image, filter_iterations);
		}
	}
	if (color_inverse == true) {
		filtered_image.negate();
	}
	save_path += "/prepared_map.png";
	filtered_image.write(save_path);
	return 0;
}