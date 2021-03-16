/**
	Objective: Reproduce the scene with the falling and bouncing sphere.

	- Complete the function "display_scene" to compute the expected trajectory
*/


#include "vcl/vcl.hpp"
#include "simulation.hpp"
#include "marching_cubes.hpp"

#include <iostream>

using namespace vcl;

// ****************************************** //
// Structure specific to the current scene
// ****************************************** //

struct user_interaction_parameters {
	vec2 mouse_prev;
	bool cursor_on_gui;
	bool display_frame = true;
	bool display_trajectory = true;
};
user_interaction_parameters user; // Variable used to store user interaction and GUI parameter

struct scene_environment
{
	camera_around_center camera;
	mat4 projection;
	vec3 light;
};
scene_environment scene; // Generic elements of the scene (camera, light, etc.)



sph_parameters_structure sph_parameters;

// ****************************************** //
// Functions signatures
// ****************************************** //

// Callback functions
void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
void window_size_callback(GLFWwindow* window, int width, int height);


void initialize_data(); // Initialize the data of this scene
void create_new_particle(float current_time); // create a new particle with speed towards the waterfall
void display_scene(float current_time);
void remove_old_particles(float current_time); // remove particle too old 
mesh create_fond(); // for the bottom of the waterfall
mesh create_terrain(int k); // for the grass around the waterfall

// the 3 following are for the waterfall ground
vec3 evaluate_xyz(float x, float y); // given x and y, compute the correct x,y and z 
float evaluate_z(float x, float y);// given x and y, compute the correct  z 
float evaluate_y(float x); // given x  compute the correct y

// for the ground, grass around the waterfall
vec3 evaluate_z_terrain(float x, float y); 


// marching cube render
vec3 grid_center = { 0,-2,-2 };
int grid_n = 20;
vec3 from_grid_to_world(vec3 grid_coord, vec3 center_grid, float step,int part);
// according to the part considered (before waterfall, waterfall or lake), returns the world coordinates according to the grid's
vec3 from_world_to_grid(vec3 world_coord, vec3 center_grid, float step);
float get_density(float x, float y, float z, int grid_size); // to create the values for the marching cube
vec3 get_density_2(float x, float y, float z, int grid_size,int part); // for the coloring of our surface (can't use the same as above)
void compute_water(int part); // for a given part, compute the mesh according to the particles and the marching cube algorithm

// ****************************************** //
// Global variables
// ****************************************** //

std::vector<particle_element> particles; // The container of the active particles

mesh_drawable waterfall;
mesh_drawable rocks;
mesh_drawable trees;
mesh_drawable sphere_particle;
mesh_drawable quad;
mesh_drawable skybox;

std::vector<mesh_drawable> terrain; // the right and left part

timer_event_periodic timer_pop(0.1f);  // Timer with periodic event
grid_3D<int> x; // for the marching cube algorithm
mesh_drawable water;

timer_basic timer;



// ****************************************** //
// Functions definitions
// ****************************************** //

// Main function with creation of the scene and animation loop
int main(int, char* argv[])
{
	std::cout << "Run " << argv[0] << std::endl;

	// create GLFW window and initialize OpenGL
	GLFWwindow* window = create_window(1280, 1024);
	window_size_callback(window, 1280, 1024);
	std::cout << opengl_info_display() << std::endl;;

	imgui_init(window); // Initialize GUI library

	// Set GLFW callback functions
	glfwSetCursorPosCallback(window, mouse_move_callback);
	glfwSetWindowSizeCallback(window, window_size_callback);

	std::cout << "Initialize data ..." << std::endl;
	initialize_data();

	std::cout << "Start animation loop ..." << std::endl;
	timer.start();
	timer_pop.start();
	glEnable(GL_DEPTH_TEST);
	while (!glfwWindowShouldClose(window))
	{
		scene.light = scene.camera.position();
		timer.update();
		timer_pop.update();

		// Clear screen
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glClear(GL_DEPTH_BUFFER_BIT);

		// Create GUI interface for the current frame
		imgui_create_frame();
		ImGui::Begin("GUI", NULL, ImGuiWindowFlags_AlwaysAutoResize);
		ImGui::Checkbox("Display frame", &user.display_frame);
		ImGui::SliderFloat("Time Scale", &timer.scale, 0.0f, 2.0f, "%.1f");

		float const dt = 0.005f * timer.scale;
		simulate(dt, particles, sph_parameters);

		// ****************************************** //
		// Specific calls of this scene
		// ****************************************** //

		// if there is a periodic event, insert a new particle
		if (timer_pop.event)
			create_new_particle(timer_pop.t);

		remove_old_particles(timer_pop.t);

		// Display the scene (includes the computation of the particle position at current time)
		display_scene(timer_pop.t);

		// ****************************************** //
		// ****************************************** //


		// Display GUI
		ImGui::End();
		imgui_render_frame(window);

		// Swap buffer and handle GLFW events
		glfwSwapBuffers(window);
		glfwPollEvents();
	}


	imgui_cleanup();
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

void initialize_data()
{
	// Load and set the common shaders
	GLuint const shader_mesh = opengl_create_shader_program(opengl_shader_preset("mesh_vertex"), opengl_shader_preset("mesh_fragment"));
	GLuint const shader_single_color = opengl_create_shader_program(opengl_shader_preset("single_color_vertex"), opengl_shader_preset("single_color_fragment"));
	GLuint const texture_white = opengl_texture_to_gpu(image_raw{ 1,1,image_color_type::rgba,{255,255,255,255} });

	mesh_drawable::default_shader = shader_mesh;
	mesh_drawable::default_texture = texture_white;
	curve_drawable::default_shader = shader_single_color;

	scene.camera.look_at({ 2,3,2 }, { 0,0,0 }, { 0,0,1 });

	sphere_particle = mesh_drawable(mesh_primitive_sphere());
	sphere_particle.transform.scale = 0.05f;

	
	waterfall = mesh_drawable(create_fond());
	waterfall.texture = opengl_texture_to_gpu(image_load_png("../assets/rocks.png"));

	terrain.push_back(mesh_drawable(create_terrain(1)));
	terrain.push_back(mesh_drawable(create_terrain(-1)));
	terrain[0].texture = opengl_texture_to_gpu(image_load_png("../assets/ground.png"));
	terrain[1].texture = opengl_texture_to_gpu(image_load_png("../assets/ground.png"));

	rocks = mesh_drawable(mesh_load_file_obj("../assets/Rock_1.obj"));
	rocks.texture = opengl_texture_to_gpu(image_load_png("../assets/textures/Rock_1_Base_Color.png"));

	trees = mesh_drawable(mesh_load_file_obj("../assets/Tree.obj"));
	trees.texture = opengl_texture_to_gpu(image_load_png("../assets/bark_0021.png"));

	mesh sky = mesh_primitive_cube({ 0,0,0 }, 20);

	
	skybox = mesh_drawable(sky);
	// skybox.texture = opengl_texture_to_gpu(image_load_png("../assets/skybox.png"));
	create_new_particle(0.0f);
	// compute_water();



}

mesh create_fond() {
	// surface
	float waterfall_height = 0.8f;
	float winning_height = 0.2f;
	int refine = 100;
	float lake_radius = 1.0f;
	float profondeur_max = -1.0f;
	float river_deep = -0.2f;
	vec2 center_lake = { -1.0f,0.0f };
	mesh fond = mesh_primitive_grid({ -2,-1,0 }, { 4,-1,0 }, { 4,1,0 }, { -2,1,0 }, refine * 3, refine);
	for (size_t i = 0; i < refine * 3; i++) {
		for (size_t j = 0; j < refine; j++) {
			// the river before the waterfall
			// waterfall between -2 and 0
			// river between 0 and 4

			// up, especially close to the arrival aka between 0 and 1
			float x = fond.position[i * refine + j].x;
			float y = fond.position[i * refine + j].y; 
			fond.position[i * refine + j] = evaluate_xyz(x, y);
		}
	}
	
	return fond;
}

mesh create_terrain(int k) {
	int refine = 100;
	mesh fond;
	float waterfall_height = 0.8f;
	float winning_height = 0.2f;
	float lake_radius = 1.0f;
	float profondeur_max = -1.0f;
	float river_deep = -0.2f;
	vec2 center_lake = { -1.0f,0.0f };
	if (k == 1) // left 
	{
		fond = mesh_primitive_grid({ -2,0,0 }, { 4,0,0 }, { 4,3.8f,0 }, { -2,3.8f,0 }, refine * 3, refine);


		for (size_t i = 0; i < refine * 3; i++) {
			for (size_t j = 0; j < refine; j++) {
				float x = fond.position[i * refine + j].x;
				float y = fond.position[i * refine + j].y;
				fond.position[i * refine + j] = evaluate_z_terrain(x, y);
			}
		}
	}
	if (k == -1) {
		fond = mesh_primitive_grid({ -2,-0.0001,0 }, { 4,-0.0001,0 }, { 4,-3.8f,0 }, { -2,-3.8f,0 }, refine * 3, refine);

		for (size_t i = 0; i < refine * 3; i++) {
			for (size_t j = 0; j < refine; j++) {
				float x = fond.position[i * refine + j].x;
				float y = fond.position[i * refine + j].y;
				fond.position[i * refine + j] = evaluate_z_terrain(x, y);
			}
		}
	}
	return fond;

}

void display_scene(float current_time)
{
	draw(waterfall, scene);
	draw(terrain[0], scene);
	draw(terrain[1], scene);

	float scale = 0.003f;
	float translate = 10.0f * scale;
	rocks.transform.scale = scale;
	rocks.transform.translate = evaluate_xyz(3.0f, 0.0f) + vec3({0, 0, translate});
	draw(rocks, scene);

	trees.transform.translate = evaluate_z_terrain(2.0, 3.0f);
	trees.transform.rotate = rotation({ 1,0,0 }, 1.57f);
	trees.transform.scale = 0.1f;
	draw(trees, scene);

	for (size_t k = 0; k < particles.size(); ++k) {
		vec3 const& p = particles[k].p;
		sphere_particle.transform.translate = p;
		float color = 0;
		float d = 0.15f;
		for (size_t i = 0; i < particles.size(); i++) {
			if (i == k) {
				continue;
			}
			float const r = norm(p - particles[i].p) / d;
			color += 0.25f * std::exp(-r * r);
		}
		sphere_particle.shading.color = vec3(clamp(1 - color, 0, 1), clamp(1 - color, 0, 1), 1);
		draw(sphere_particle, scene);

		
	}

	compute_water(0);
	draw(water, scene);
	compute_water(1);
	draw(water, scene);
	compute_water(2);
	draw(water, scene);

	// draw(skybox, scene);
}

vec3 evaluate_xyz(float x, float y) {

	float waterfall_height = 0.8f;
	float winning_height = 0.4f;
	int refine = 100;
	float lake_radius = 1.0f;
	float profondeur_max = -1.0f;
	float river_deep = -0.2f;
	vec2 center_lake = { -1.0f,0.0f };
	std::vector<float> res;

	

	float z = waterfall_height * (std::atan(10.0f * (-0.2f + x)) - std::atan(-2.0f));
	if (x > 0) {
		// less wide close to the fall
		if (x > 0 && x < 1.0f) {
			x += x * (1 - x) * noise_perlin(vec2(x, y), 8, 0.8f);
		}
		y = y / (1.0f + std::min(std::pow(x, 2), 3.0f));
		// the river is going down slowly at first
		z += x * winning_height;


		// the river is deeper on the inside
		float max_width = 1.0f / (1.0f + std::min(std::pow(x, 2), 3.0f));

		if (max_width - std::abs(y) < max_width / 3.0f) {
			z += river_deep * (max_width - std::abs(y)) * 3.0f / max_width * std::min(1.0, x / 0.5);
		}
		else {
			z += river_deep * std::min(1.0, x / 0.5) + 0.03f*noise_perlin(vec2({ x,y }), 5, 0.6f);
		}




	}

	else {
		// in the lake centered around -1,0
		z = profondeur_max * (1 - std::max(std::pow(x * 0.9f - center_lake[0], 2), std::pow(y - center_lake[1], 2)));
		y = y * (2.0f - std::pow(x - center_lake[0], 2));
	}
	// perlin
	float noise = 0.1f * noise_perlin(vec2({ x,y }), 5, 0.3f);
	z += noise;
	return { x,y,z };
}

float evaluate_z(float x, float y) {

	float waterfall_height = 0.8f;
	float winning_height = 0.4f;
	int refine = 100;
	float lake_radius = 1.0f;
	float profondeur_max = -1.0f;
	float river_deep = -0.2f;
	vec2 center_lake = { -1.0f,0.0f };
	std::vector<float> res;


	float z = waterfall_height * (std::atan(10.0f * (-0.2f + x)) - std::atan(-2.0f));
	if (x > 0) {

		if (x > 0 && x < 1.0f) {
			x += x * (1 - x) * noise_perlin(vec2(x, y), 8, 0.8f);
		}
		// the river is going down slowly at first
		z += x * winning_height;

		
		// the river is deeper on the inside
		float max_width = 1.0f / (1.0f + std::min(std::pow(x, 2), 3.0f));

		if (max_width - std::abs(y) < max_width / 3.0f) {
			z += river_deep * (max_width - std::abs(y)) * 3.0f / max_width * std::min(1.0, x / 0.5);
		}
		else {
			z += river_deep * std::min(1.0, x / 0.5) + 0.03f * noise_perlin(vec2({ x,y }), 5, 0.6f);
		}


	}
	else {
		// in the lake centered around -1,-1
		z = profondeur_max * (1 - std::max(std::pow(x * 0.9f - center_lake[0], 2), std::pow(y / (2.0f - std::pow(x - center_lake[0], 2)) - center_lake[1], 2)));
	}
	// perlin
	float noise = 0.1f * noise_perlin(vec2({ x,y }), 5, 0.3f);
	z += noise;
	return z;
}

float evaluate_y(float x) {
	float waterfall_height = 0.8f;
	float winning_height = 0.4f;
	int refine = 100;
	float lake_radius = 1.0f;
	float profondeur_max = -1.0f;
	float river_deep = -0.2f;
	vec2 center_lake = { -1.0f,0.0f };
	float y = 0.0f;

	if (x > 0) {
		// less wide close to the fall
		
		y = 1.0f / (1.0f + std::min(std::pow(x, 2), 3.0f));
	}
	else {
		
		// in the lake centered around -1,-1
		y = (2.0f - std::pow(x - center_lake[0], 2));
	}
	return y;
}

vec3 evaluate_z_terrain(float x, float y) {
	float waterfall_height = 0.8f;
	float winning_height = 0.4f;
	float lake_radius = 1.0f;
	float profondeur_max = -1.0f;
	float river_deep = -0.2f;
	vec2 center_lake = { -1.0f,0.0f };
	float z;
	if (y >= 0) {
		z = waterfall_height * (std::atan(10.0f * (-0.2f + x)) - std::atan(-2.0f));
		if (x > 0) {
			if (x > 0 && x < 1.0f) {
				x += x * (1 - x) * noise_perlin(vec2(x, y+1.0f), 8, 0.8f);
			}
			// less wide close to the fall
			y = std::min(10.0f, y + 1.0f / (1.0f + std::min(std::pow(x, 2), 3.0f)));		

			// the river is going down slowly at first
			z += x * winning_height;
		}
		else {
			// in the lake centered around -1,-1
			float coef = 1.0f - y / 4.0f;
			z = coef * profondeur_max * (1 - std::max(std::pow(x * 0.9f - center_lake[0], 2), std::pow(1.0f - center_lake[1], 2)));
			y = std::min(10.0f, y + 1.0f * (2.0f - std::pow(x - center_lake[0], 2)));
		}
	}
	else {
		z = waterfall_height * (std::atan(10.0f * (-0.2f + x)) - std::atan(-2.0f));
		if (x > 0) {
			if (x > 0 && x < 1.0f) {
				x += x * (1 - x) * noise_perlin(vec2(x, y-1.0f), 8, 0.8f);
			}
			// less wide close to the fall
			y = std::max(-10.0f, y - 1.0f / (1.0f + std::min(std::pow(x, 2), 3.0f)));
			// the river is going down slowly at first
			z += x * winning_height;
		}
		else {
			// in the lake centered around -1,-1
			float coef = 1.0f - y / 4.0f;
			z = coef * profondeur_max * (1 - std::max(std::pow(x * 0.9f - center_lake[0], 2), std::pow(1.0f - center_lake[1], 2)));
			y = std::max(-10.0f, y - 1.0f * (2.0f - std::pow(x - center_lake[0], 2)));
		}
	}
	

	// perlin
	float noise = 0.1f*noise_perlin(vec2({ x,y }), 5, 0.3f);
	z += noise;

	// collines 
	std::vector<float> heights = { 0.4f, 0.3f };
	std::vector<float> std = { 0.5f, 0.6f };
	std::vector<vec2> centers = { {2.0f,-2.0f}, {2.0f,2.0f} };
	for (int k = 0; k < heights.size(); k++) {
		z += heights[k] * std::exp(-std::pow(norm(vec2({ x,y }) - centers[k]), 2) / std::pow(std[k], 2));
	}
	return { x,y,z };
}

void create_new_particle(float current_time) {
	particle_element new_particle;
	new_particle.p = evaluate_xyz(3.0f,0.0f) + vec3(0,rand_interval(-0.1f,0.1f),0.0f);
	new_particle.v = { -0.2f,0.0f,0.0f }; // going down the cascade
	new_particle.t = current_time;
	particles.push_back(new_particle);
}

void remove_old_particles(float current_time) {
	for (auto it = particles.begin(); it != particles.end();)
	{
		// if a particle is too old, remove it
		if (current_time - it->t > 200) { // change time I guess
			it = particles.erase(it);
		}
		// Go to the next particle if we are not already on the last one
		if (it != particles.end())
			++it;
	}
}

float get_density(float x, float y, float z, int grid_size) {
	float density = 0.0f;
	vec3 p0 = vec3(x, y, z);
	float const d = 0.05f;
	for (int i = 0; i < particles.size(); i++) {
		vec3 const& pi = particles[i].p;
		float const r = norm(pi - p0) / d;
		density += 0.25f * std::exp(-r * r);
	}
	return density;
}

vec3 get_density_2(float x, float y, float z, int grid_size,int part) {
	float density = 0.0f;
	vec3 p0 = vec3(x, y, z);
	// quite clear
	if (part == 0 || part == 1 || part == 2 ) {
		float const d = 0.2f;
		for (int i = 0; i < particles.size(); i++) {
			vec3 const& pi = particles[i].p;
			float const r = norm(pi - p0) / d;
			density += 0.35f * std::exp(-r * r);
		}

		vec3 color = vec3(clamp(1 - density, 0, 1), clamp(1 - density, 0, 1), 1);

		return color * 2.5f;
	}
	// darker
	if (part == 1) {
		float const d = 0.3f;
		for (int i = 0; i < particles.size(); i++) {
			vec3 const& pi = particles[i].p;
			float const r = norm(pi - p0) / d;
			density += 0.15f * std::exp(-r * r);
		}
		density -= 0.4f;

		vec3 color = vec3(clamp(1 - density, 0, 1), clamp(1 - density, 0, 1), 1 - density / 3.0f);
		if (density < 0.2f) {
			color = { 0.9,0.9,0.9 };
		}
		return color;
	}
	
	return vec3(0,0,0);
}

vec3 from_grid_to_world(vec3 grid_coord, vec3 center_grid, float step, int part) {
	// if cube
	// return grid_coord / step + center_grid;
	
	if (part == 0) { // before the water fall
		// start above the waterfall ie 4.2 > x > 1 and y between -1 and 1 and z between 2.9 and 3.9
		float x = 1.0f + grid_coord.x / (step - 1) * 3.2f;
		float y = -1.0f + grid_coord.y / (step - 1) * 2.0f;
		float z = 2.2f + grid_coord.z / (step - 1) * 1.2f;
		return vec3(x, y, z);
	}
	if (part == 1) { // the waterfall
		// start above the waterfall ie 1 > x > 0 and y between -1 and 1 and z between 2.9 and 0
		float x =-1.5f +  grid_coord.x / (step - 1)*2.5f;
		float y = -1.0f + grid_coord.y / (step - 1) * 2.0f;
		float z = grid_coord.z / (step - 1) * 3.0f;
		return vec3(x, y, z);
	}
	if (part == 2) { // the lake
		// start above the waterfall ie 0 > x > -4 and y between -2 and 2 and z between 0.5 and 0
		float x = -4.0f + grid_coord.x / (step - 1)*4.0f;
		float y = -2.0f + grid_coord.y / (step - 1) * 4.0f;
		float z = -2.0f + grid_coord.z / (step - 1) * 2.5f;
		return vec3(x, y, z);
	}
	

}

vec3 from_world_to_grid(vec3 world_coord, vec3 center_grid, float step) {
	return (world_coord - center_grid) * step;
}

void compute_water(int part) {
	vcl::grid_3D<float> grid;
	grid.resize(grid_n, grid_n, grid_n);
	for (int i = 0; i < grid_n; i++) {
		for (int j = 0; j < grid_n; j++) {
			for (int k = 0; k < grid_n; k++) {
				vec3 pos = from_grid_to_world({ i,j,k }, grid_center, grid_n,part);
				grid(i, j, k) = get_density(pos.x, pos.y, pos.z, grid_n);
				// std::cout << pos << " " << grid(i, j, k)  << std::endl;
			}
		}
	}

	marching_cubes marching;
	mesh sur = marching.marching_cube(grid, grid_n, 0.001);
	sur.color.resize(sur.position.size());
	for (int i = 0; i < sur.position.size(); i++) {
		sur.position(i) = from_grid_to_world(sur.position(i), grid_center, grid_n,part);
		sur.color(i) = get_density_2(sur.position(i).x, sur.position(i).y, sur.position(i).z, grid_n,part); ;

	}

	sur.uv.resize(sur.position.size());

	if (sur.position.size() != 0) {
		water = mesh_drawable(sur);
	}
	else {
		water = mesh_drawable(mesh_primitive_cube({ -1,-1,-1 },0.001f));
	}
	
}


// Function called every time the screen is resized
void window_size_callback(GLFWwindow*, int width, int height)
{
	glViewport(0, 0, width, height);
	float const aspect = width / static_cast<float>(height);
	scene.projection = projection_perspective(50.0f * pi / 180.0f, aspect, 0.1f, 100.0f);
}

// Function called every time the mouse is moved
void mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
	vec2 const  p1 = glfw_get_mouse_cursor(window, xpos, ypos);
	vec2 const& p0 = user.mouse_prev;

	glfw_state state = glfw_current_state(window);
	user.cursor_on_gui = ImGui::IsAnyWindowFocused();

	// Handle camera rotation
	auto& camera = scene.camera;
	if (!user.cursor_on_gui) {
		if (state.mouse_click_left && !state.key_ctrl)
			scene.camera.manipulator_rotate_trackball(p0, p1);
		if (state.mouse_click_left && state.key_ctrl)
			camera.manipulator_translate_in_plane(p1 - p0);
		if (state.mouse_click_right)
			camera.manipulator_scale_distance_to_center((p1 - p0).y);
	}


	user.mouse_prev = p1;
}

// Uniform data used when displaying an object in this scene
void opengl_uniform(GLuint shader, scene_environment const& current_scene)
{
	opengl_uniform(shader, "projection", current_scene.projection);
	opengl_uniform(shader, "view", scene.camera.matrix_view());
	opengl_uniform(shader, "light", scene.light, false);
}






