#include "simulation.hpp"

using namespace vcl;

// to handle colision with nature
float evaluate_y_s(float x) {
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

float evaluate_z_s(float x, float y) {

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

// Convert a density value to a pressure
float density_to_pressure(float rho, float rho0, float stiffness)
{
	return stiffness*(rho-rho0);
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
	float const r = norm(p_i-p_j);
    assert_vcl_no_msg(r<=h);
	return 315.0/(64.0*3.14159f*std::pow(h,9)) * std::pow(h*h-r*r, 3.0f);
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
    for(size_t i=0; i<N; ++i)
        particles[i].pressure = density_to_pressure(particles[i].rho, rho0, stiffness);
}

// Compute the forces and update the acceleration of the particles
void update_force(std::vector<particle_element>& particles, float h, float m, float nu)
{
	// gravity
    const size_t N = particles.size();
    for (size_t i = 0; i < N; ++i) {
        particles[i].f = m * vec3{ 0,0,-20.0f }; // increase gravity
        for (size_t j = 0; j < N; ++j){
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

                force_pressure = -m / rho_i *  (pressure_i + pressure_j) / (2 * rho_j) * W_gradient_pressure(pi, pj, h);
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
	for(size_t k=0; k<N; ++k)
	{
		vec3& p = particles[k].p;
		vec3& v = particles[k].v;
		vec3& f = particles[k].f;
        //float const rho = particles[k].rho;

		v = (1-damping)*v + dt*f/m;//rho;
		p = p + dt*v;
	}


	// Collision
    float const epsilon = 1e-3f;
    for (size_t k = 0; k < N; ++k)
    {
        vec3& p = particles[k].p;
        vec3& v = particles[k].v;

        // small perturbation to avoid alignment

        float y_limit = evaluate_y_s(p.x);

        if (p.y > y_limit) {
            p.y = y_limit - epsilon * rand_interval();
            v.y *= -0.5f;
        }

        if (p.y < -y_limit) {
            p.y = -y_limit + epsilon * rand_interval();
            v.y *= -0.5f;
        }
        if (evaluate_z_s(p.x, p.y) > p.z) {
            p.z = evaluate_z_s(p.x, p.y) + epsilon * rand_interval();
            v.z *= -0.5f;
        }

        if (p.x > 1.0f && p.z > evaluate_z_s(p.x, p.y) + 0.05f) {
            p.z = 0.2f + evaluate_z_s(p.x, p.y) - epsilon * rand_interval();
            v.z *= -0.5f;
        }


        if (p.x < -2) { p.x = -2 + epsilon * rand_interval();  v.x *= -0.5f; }
        if (p.x > 4) { p.x = 4 - epsilon * rand_interval();  v.x *= -0.5f; }

    }

}