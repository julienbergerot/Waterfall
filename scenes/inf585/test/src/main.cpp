/**
	Objective: Reproduce the scene with the falling and bouncing sphere.

	- Complete the function "display_scene" to compute the expected trajectory
*/


#include "vcl/vcl.hpp"
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

struct particle_element
{
	vcl::vec3 p; // Position
	vcl::vec3 v; // Speed
	vcl::vec3 f; // Force
	float t;
	float rho;      // density at this particle position
	float pressure; // pressure at this particle position
	particle_element() : p{ 0,0,0 }, v{ 0,0,0 }, f{ 0,0,0 }, t(0),rho(0), pressure(0) {}
};

// SPH simulation parameters
struct sph_parameters_structure
{
	// Influence distance of a particle (size of the kernel)
	float h = 0.12f;
	// Rest density (normalized to 1 - real unit should be 1000kg/m^2)
	float rho0 = 1;
	// Total mass of a particle (consider rho0 h^2)
	float m = rho0 * h * h;
	// viscosity parameter
	float nu = 0.05f;
	// Stiffness converting density to pressure
	float stiffness = 0.3f;
};

sph_parameters_structure sph_parameters;

// ****************************************** //
// Functions signatures
// ****************************************** //

// Callback functions
void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
void window_size_callback(GLFWwindow* window, int width, int height);


void initialize_data(); // Initialize the data of this scene
void create_new_particle(float current_time);
void display_scene(float current_time);
void remove_old_particles(float current_time);
mesh create_fond();
mesh create_terrain(int k);
vec3 evaluate_xyz(float x, float y);
float evaluate_z(float x, float y);
float evaluate_y(float x);
vec3 evaluate_z_terrain(float x, float y);

// sph 
float density_to_pressure(float rho, float rho0, float stiffness);
float W_laplacian_viscosity(vec3 const& p_i, vec3 const& p_j, float h);
vec3 W_gradient_pressure(vec3 const& p_i, vec3 const& p_j, float h);
float W_density(vec3 const& p_i, const vec3& p_j, float h);
void update_density(buffer<particle_element>& particles, float h, float m);
void update_pressure(buffer<particle_element>& particles, float rho0, float stiffness);
void update_force(buffer<particle_element>& particles, float h, float m, float nu);

void simulate(float dt, std::vector<particle_element>& particles, sph_parameters_structure const& sph_parameters);

mesh initialize_sph();

// ****************************************** //
// Global variables
// ****************************************** //

std::vector<particle_element> particles; // The container of the active particles

mesh_drawable waterfall;
mesh_drawable rocks;
mesh_drawable trees;
mesh_drawable sphere_particle;
mesh_drawable quad;
std::vector<mesh_drawable> terrain;
vec3 starting_pos = { 4.0f,0,0.8f*(std::atan(10.0f * (-0.5f + 4.0f)) - std::atan(-5.0f)) + 0.2f * 4.0f };

timer_event_periodic timer_pop(0.05f);  // Timer with periodic event

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
}

vec3 evaluate_xyz(float x, float y) {

	float waterfall_height = 0.8f;
	float winning_height = 0.2f;
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
	float winning_height = 0.2f;
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
	float winning_height = 0.2f;
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
	float winning_height = 0.2f;
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
	new_particle.p = evaluate_xyz(4.0f,0.0f) + vec3(0,rand_interval(-0.1f,0.1f),0.0f);
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

// Convert a density value to a pressure
float density_to_pressure(float rho, float rho0, float stiffness)
{
	return stiffness * (rho - rho0);
}

float W_laplacian_viscosity(vec3 const& p_i, vec3 const& p_j, float h)
{
	// To do ...
	//  Fill it with laplacian of W_viscosity
	float const r = norm(p_i - p_j);
	assert_vcl_no_msg(r <= h);
	return 45 / (3.14159f * std::pow(h, 6.0f)) * (h - r);
}

vec3 W_gradient_pressure(vec3 const& p_i, vec3 const& p_j, float h)
{
	// To do ...
	//  Fill it with gradient of W_spiky
	float const r = norm(p_i - p_j);
	assert_vcl_no_msg(r <= h);
	return -45 / (3.14159f * std::pow(h, 6.0f)) * std::pow(h - r, 2) * (p_i - p_j) / r;
}

float W_density(vec3 const& p_i, const vec3& p_j, float h)
{
	float const r = norm(p_i - p_j);
	assert_vcl_no_msg(r <= h);
	return 315.0 / (64.0 * 3.14159f * std::pow(h, 9)) * std::pow(h * h - r * r, 3.0f);
}


void update_density(std::vector<particle_element>& particles, float h, float m)
{
	// To do: Compute the density value (particles[i].rho) at each particle position
	//  rho_i = \sum_j m W_density(pi,pj)
	size_t const N = particles.size();
	for (size_t i = 0; i < N; ++i) {
		particles[i].rho = 0.0f;
		for (size_t j = 0; j < N; j++) {
			if (norm(particles[i].p - particles[j].p) < h) {
				particles[i].rho += m * W_density(particles[i].p, particles[j].p, h); // to be modified
			}
		}
	}


}

// Convert the particle density to pressure
void update_pressure(std::vector<particle_element>& particles, float rho0, float stiffness)
{
	const size_t N = particles.size();
	for (size_t i = 0; i < N; ++i)
		particles[i].pressure = density_to_pressure(particles[i].rho, rho0, stiffness);
}

// Compute the forces and update the acceleration of the particles
void update_force(std::vector<particle_element>& particles, float h, float m, float nu)
{
	// gravity
	const size_t N = particles.size();
	for (size_t i = 0; i < N; ++i) {
		particles[i].f = m * vec3{ 0,0,-9.81f };
		for (size_t j = 0; j < N; ++j) {
			if (i == j) {
				continue;
			}
			const vec3& pi = particles[i].p;
			const vec3& pj = particles[j].p;
			float r = norm(pi - pj);
			if (r < h) {
				const vec3& vi = particles[i].v;
				const vec3& vj = particles[j].v;

				const float pressure_i = particles[i].pressure;
				const float pressure_j = particles[j].pressure;

				const float rho_i = particles[i].rho;
				const float rho_j = particles[j].rho;

				vec3 force_pressure = { 0,0,0 };
				vec3 force_viscosity = { 0,0,0 };

				force_pressure = -m / rho_i * (pressure_i + pressure_j) / (2 * rho_j) * W_gradient_pressure(pi, pj, h);
				force_viscosity = m * nu * m * (vj - vi) / rho_j * W_laplacian_viscosity(pi, pj, h);


				particles[i].f += force_pressure + force_viscosity;

			}

		}
	}


}

void simulate(float dt, std::vector<particle_element>& particles, sph_parameters_structure const& sph_parameters)
{

	// Update values
	update_density(particles, sph_parameters.h, sph_parameters.m);                   // First compute updated density
	update_pressure(particles, sph_parameters.rho0, sph_parameters.stiffness);       // Compute associated pressure
	update_force(particles, sph_parameters.h, sph_parameters.m, sph_parameters.nu);  // Update forces

	// Numerical integration
	float const damping = 0.005f;
	size_t const N = particles.size();
	float const m = sph_parameters.m;
	for (size_t k = 0; k < N; ++k)
	{
		vec3& p = particles[k].p;
		vec3& v = particles[k].v;
		vec3& f = particles[k].f;
		//float const rho = particles[k].rho;

		v = (1 - damping) * v + dt * f / m;//rho;
		p = p + dt * v;
	}


	// Collision
	float const epsilon = 1e-3f;
	for (size_t k = 0; k < N; ++k)
	{
		vec3& p = particles[k].p;
		vec3& v = particles[k].v;

		// small perturbation to avoid alignment
		
		float y_limit = evaluate_y(p.x);
		
		if (p.y > y_limit) {
			p.y = y_limit - epsilon * rand_interval();
			v.y *= -0.5f;
		}
		
		if (p.y < -y_limit) {
			p.y = -y_limit + epsilon * rand_interval();
			v.y *= -0.5f;
		}
		if (evaluate_z(p.x, p.y) > p.z) {
			p.z = evaluate_z(p.x, p.y) + epsilon * rand_interval();
			v.z *= -0.5f;
		}

		if (p.x > 1.0f && p.z > evaluate_z(p.x, p.y) + 0.2f) {
			p.z = 0.2f + evaluate_z(p.x, p.y) - epsilon * rand_interval();
			v.z *= -0.5f;
		}


		if (p.x < -2) { p.x = -2 + epsilon * rand_interval();  v.x *= -0.5f; }
		if (p.x > 4) { p.x = 4 - epsilon * rand_interval();  v.x *= -0.5f; }

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






