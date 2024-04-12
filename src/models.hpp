#pragma once

#include <vector>
#include <givr.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>

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
		struct grid {
			float scale;
			float cube_width;
			std::vector<primatives::boid>* boids;
			std::vector<std::vector<size_t>> buckets;
			std::vector<size_t>& get_buckets(float x, float y, float z){
				int i = floor((x + cube_width)/scale);
				int j = floor((y + cube_width) / scale);
				int k = floor((z + cube_width) / scale);
				int n = cube_width * 2;
				return buckets[i+(n*j)+(n*n*k)];
			}
		};
		//Boids in simulation
		struct boid {
			int id;
			glm::vec3 p = glm::vec3(0.f);
			glm::vec3 v = glm::vec3(0.f);
			glm::vec3 f = glm::vec3(0.f);
			void integrate(float dt){
				v = v + f*dt;
				float max_speed = 25;
				v = {std::min(v.x, max_speed), std::min(v.y, max_speed), std::min(v.z, max_speed)};
				p = p + v*dt;	
			}
			void separation_force(float k, primatives::boid b2, glm::vec3 p_ij, float d){
				if (d >= 0) {
					d = std::max(d, 0.00001f);
				}
				else {
					d = std::min(d, -0.00001f);
				}
				glm::vec3 f_sep = (k / d) * glm::normalize(p_ij);
				/*for (float p_i : {f_sep.x, f_sep.y, f_sep.z}) {
					if (p_i > 10000 || p_i < -10000) {
						std::cout << "id: " << id << " f_sep: [ " << f_sep.x << ", " << f_sep.y << ", " << f_sep.z << ",]" << std::endl;
						break;
					}
				}*/
				f += f_sep;
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
				float multiplier = ((glm::dot(v, v)) / r);
				float cap = 10000.f;
				if (multiplier < 0) {
					multiplier = std::max(-cap, multiplier);
				}
				else {
					multiplier = std::min(cap, multiplier);
				}
				glm::vec3 fa = multiplier*fa_norm;
				/*for (float p_i : {fa.x, fa.y, fa.z}) {
					if (p_i > 10000 || p_i < -10000) {
						std::cout << "id: " << id << " multiplier: " << multiplier << "fa_norm: [" << fa_norm.x << ", " << fa_norm.y << ", " << fa_norm.z << ", ] " << std::endl;
						std::cout << "id: " << id << " fa: [ " << fa.x << ", " << fa.y << ", " << fa.z << ",]" << std::endl;
						break;
					}
				}*/
				f+=fa;
			}
			void repulsion_force(primatives::plane plane, float d){
				float k = 5;
				float multiplier = std::min(5000.f, k/(d*d));
				//if (multiplier == INFINITY) multiplier = 10000.f;
				glm::vec3 f_r = plane.normal*(k/(d*d));
				/*for (float p_i : {f_r.x, f_r.y, f_r.z}) {
					if (p_i > 10000 || p_i < -10000) {
						std::cout << "d: " << d << " d*d: " << d * d << " k: " << k << " k/d*d: " << k / d * d << std::endl;
						std::cout << "id: " << id << " f_r: [ " << f_r.x << ", " << f_r.y << ", " << f_r.z << ",]" << std::endl;
						break;
					}
				}*/
				f+=f_r;
			}
			void penalty_force(primatives::plane plane){
					float k = 10;
					float d = glm::dot(plane.normal, p-plane.point);	
					glm::vec3 f_coll = -k*d*plane.normal;
					/*for (float p_i : {f_coll.x, f_coll.y, f_coll.z}) {
						if (p_i > 10000 || p_i < -10000) {
							std::cout << "id: " << id << " f_coll: [ " << f_coll.x << ", " << f_coll.y << ", " << f_coll.z << ",]" << std::endl;
							std::cout << "d: " << d << " k: " << k << " -k*d: " << -k*d << std::endl;
							break;
						}
					}*/
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

			//Simulation Constants (you can re-assign values here from imgui)
			glm::vec3 g = { 0.f, -9.81f, 0.f };
			size_t n_boids = 10;
			float rs, ra, rc;
			float ds, da, dc;
			float ks, ka, kc;
			int cube_width = 20;

		private:
			//Simulation Parts
			std::vector<primatives::boid> boids;
			std::vector<primatives::plane> planes;
			primatives::grid grid;
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