#pragma once

#include <vector>
#include <givr.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp
using namespace std;

namespace simulation {
	namespace primatives {
		//Walls for avoidences and simulation boundry
		struct plane {
			glm::vec3 point;
			glm::vec3 normal;
			std::string id;
			float epsilon = 1;
		};
		//Boids in simulation
		struct boid {
			int id;
			int ix;
			int iy;
			int iz;
			glm::vec3 p = glm::vec3(0.f);
			glm::vec3 v = glm::vec3(0.f);
			glm::vec3 f = glm::vec3(0.f);
			vector<vector<primatives::boid>> neighbourhood;
			void integrate(float dt){
				v = v + f*dt;
				float max_speed = 25;
				v = {std::min(v.x, max_speed), std::min(v.y, max_speed), std::min(v.z, max_speed)};
				p = p + v*dt;	
			}
			void separation_force(float k, primatives::boid b2, glm::vec3 p_ij, float d){
				f += (k/d)*glm::normalize(p_ij);
			}
			void alignment_force(float k, primatives::boid b2){
				f += k*(b2.v-v);
			}
			void cohesion_force(float k, primatives::boid b2){
				f += k*(b2.p-p);
			}
			void avoidance_force(primatives::plane plane, glm::vec3 u){
				float e = plane.epsilon;

				glm::vec3 v_norm = glm::normalize(v);
				glm::vec3 fa_norm = glm::normalize(plane.normal-glm::dot(plane.normal, v_norm)*v_norm);
				float divisor = std::max(1-glm::dot(fa_norm, plane.normal), 0.00001f);
				float r = (glm::dot(u, plane.normal)-e)/(divisor);
				glm::vec3 fa = ((glm::dot(v,v))/r)*fa_norm;
				f+=fa;
			}
			void repulsion_force(primatives::plane plane, float d){
				float k = 5;
				d = std::max(d, 0.00001f);
				glm::vec3 f_r = plane.normal*(k/(d*d));
				f+=f_r;
			}
			void penalty_force(primatives::plane plane){
					float k = 10;
					float d = glm::dot(plane.normal, p-plane.point);
					glm::vec3 f_coll = -k*d*plane.normal;
					f += f_coll;
			}
			void calc_avoidance(std::vector<primatives::plane> planes){
				for (primatives::plane& plane : planes) {
					glm::vec3 u = p - plane.point;
					float d = glm::dot(plane.normal,u)/glm::length(plane.normal);
					//If boid is moving in the direction of plane
					if (glm::dot(plane.normal, glm::normalize(v))<0){
						if (d>0.00001 && d<1) {
							// If boid is very close to the wall, apply a repulsion force
							// std::cout<< "id: " << id << " repulse"<<std::endl;
							repulsion_force(plane, d);
						} else if (d<2){
							// If boid if nearing the wall, steer away
							// std::cout<< "id: " << id << " steer"<<std::endl;
							avoidance_force(plane, u);
						}
					} if (d<0){
						// If boid exits the cube, apply a penalty force
						// std::cout<< "id: " << id << " penalty "<<std::endl;
						penalty_force(plane);
					}
				}
			}
			//Note: Mass is implicitly 1 (our choice) so Force = Acceleration
		};

		//Spheres for avoidences
		struct sphere {
			//TO-DO: Define a sphere for collision avoidence purposes
		};
	} // namespace primatives

	namespace models {
		//If you want to use a different view, change this and the one in main
		using ModelViewContext = givr::camera::ViewContext<givr::camera::TurnTableCamera, givr::camera::PerspectiveProjection>;
		// Abstract class used by all models
		class GenericModel {
		public:
			virtual void reset() = 0;
			virtual void step(float dt) = 0;
			virtual void render(const ModelViewContext& view) = 0;
		};

		//Model constructing a single spring
		class BoidsModel : public GenericModel {
		public:
			BoidsModel();
			void reset();
			
			void set_constants(const float arr[]);
			void step(float dt);
			void render(const ModelViewContext& view);
			void get_grid();
			vector<vector<primatives::boid>>  get_neighbourhood(unordered_map<int, unordered_map<int, unordered_map<int, vector<primatives::boid>>>> grid, primatives::boid boid);
			unordered_map<int, unordered_map<int, unordered_map<int, vector<primatives::boid>>>> grid;  

			//Simulation Constants (you can re-assign values here from imgui)
			glm::vec3 g = { 0.f, -9.81f, 0.f };
			size_t n_boids = 10; //need alot more eventually for full assignment
			float rs, ra;
			int rc;
			int last_rc = 0;
			float ds, da, dc;
			float ks, ka, kc;
			float t_g;
			int cube_width = 20;
			int gridmax;

		private:
			//Simulation Parts
			std::vector<primatives::boid> boids;
			std::vector<primatives::plane> planes;
			//std::vector<primatives::sphere> spheres;

			//Render
			givr::geometry::Mesh boid_geometry;
			givr::style::Phong boid_style;
			givr::InstancedRenderContext<givr::geometry::Mesh, givr::style::Phong> boid_render;

			givr::geometry::TriangleSoup wall_geometry;
			givr::style::Phong wall_style;
			givr::RenderContext<givr::geometry::TriangleSoup, givr::style::Phong> wall_render;

			//givr::geometry::Sphere sphere_geometry;
			//givr::style::Phong sphere_style;
			//givr::InstancedRenderContext<givr::geometry::TriangleSoup, givr::style::Phong> sphere_render;
		};
	} // namespace models
} // namespace simulation