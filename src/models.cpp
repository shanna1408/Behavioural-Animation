#include "models.hpp"
#include <random>
#include <unordered_map> 
#include <glm/glm.hpp>
#include <cmath>
using namespace std;

namespace simulation {
	namespace primatives {
		//No functions for structs, yet...
	}// namespace primatives

	namespace models {
		//Unless changed in main, this will call only once before the window is created
		BoidsModel::BoidsModel()
			: boid_geometry(givr::geometry::Mesh(givr::geometry::Filename("../models/dart.obj")))
			, boid_style(givr::style::Colour(1.f, 1.f, 0.f), givr::style::LightPosition(100.f, 100.f, 100.f))
			, wall_geometry()
			, wall_style(givr::style::Colour(2.f, 1.f, 1.f), givr::style::LightPosition(100.f, 100.f, 100.f))
		{
			// Reset Dynamic elements
			reset();

			planes.resize(6);
			// //Top
			planes[0].point = {0,cube_width,0};
			planes[0].normal = {0,-1,0};
			planes[0].id = "top";
			//Bottom
			planes[1].point = {0,-cube_width,0};
			planes[1].normal = {0,1,0};
			planes[1].id = "bottom";
			//Left
			planes[2].point = {-cube_width,0,0};
			planes[2].normal = {1,0,0};
			planes[2].id = "left";
			//Right
			planes[3].point = {cube_width,0,0};
			planes[3].normal = {-1,0,0};
			planes[3].id = "right";
			//Back
			planes[4].point = {0,0,-cube_width};
			planes[4].normal = {0,0,1};
			planes[4].id = "back";
			//Front
			planes[5].point = {0,0,cube_width};
			planes[5].normal = {0,0,-1};
			planes[5].id = "front";


			wall_geometry.push_back(givr::geometry::Point1(-cube_width, -cube_width, cube_width), givr::geometry::Point2(-cube_width, -cube_width, -cube_width), givr::geometry::Point3(cube_width, -cube_width, cube_width));
			wall_geometry.push_back(givr::geometry::Point1(cube_width, -cube_width, -cube_width), givr::geometry::Point2(-cube_width, -cube_width, -cube_width), givr::geometry::Point3(cube_width, -cube_width, cube_width));
			wall_geometry.push_back(givr::geometry::Point1(-cube_width, -cube_width, -cube_width), givr::geometry::Point2(-cube_width, cube_width, -cube_width), givr::geometry::Point3(cube_width, -cube_width, -cube_width));
			wall_geometry.push_back(givr::geometry::Point1(cube_width, cube_width, -cube_width), givr::geometry::Point2(-cube_width, cube_width, -cube_width), givr::geometry::Point3(cube_width, -cube_width, -cube_width));
			wall_geometry.push_back(givr::geometry::Point1(-cube_width, -cube_width, -cube_width), givr::geometry::Point2(-cube_width, cube_width, -cube_width), givr::geometry::Point3(-cube_width, -cube_width, cube_width));
			wall_geometry.push_back(givr::geometry::Point1(-cube_width, cube_width, cube_width), givr::geometry::Point2(-cube_width, cube_width, -cube_width), givr::geometry::Point3(-cube_width, -cube_width, cube_width));
			// Render
			boid_render = givr::createInstancedRenderable(boid_geometry, boid_style);
			wall_render = givr::createRenderable(wall_geometry, wall_style);
		}

		void BoidsModel::reset() {
			// static so they are persistent
			static random_device random_device;
			static mt19937 generator(random_device());
			static uniform_real_distribution<double> position_distribution(-cube_width, cube_width);
			static uniform_real_distribution<double> theta_distribution(-180., 180.);
			static uniform_real_distribution<double> phi_distribution(-90., 90.);
			static uniform_real_distribution<double> speed_distribution(5., 25.);

			boids.resize(n_boids);
			int i = 0;
			for (primatives::boid& boid : boids) {
				//Random Position in 40 x 40 x 40 cube
				boid.p.x = position_distribution(generator);
				boid.p.y = position_distribution(generator);
				boid.p.z = position_distribution(generator);
				boid.id = i;
				i++;
				boid.v = {0,0,0};
				boid.f = {0,0,0};

				//Random heading (two angles) and Random speed
				double theta = glm::radians(theta_distribution(generator));
				double phi = glm::radians(phi_distribution(generator));
				double speed = speed_distribution(generator);

				// https://stackoverflow.com/questions/30011741/3d-vector-defined-by-2-angles
				boid.v.x = speed * cos(theta) * cos(phi);
				boid.v.y = speed * sin(phi);
				boid.v.z = speed * sin(theta) * cos(phi);

			}
			get_grid();

		}

		void BoidsModel::set_constants(const float arr[]){
			ks = arr[0];
			ka = arr[1];
			kc = arr[2];
			rs = arr[3];
			ra = arr[4];
			rc = arr[5];
			ds = arr[6];
			da = arr[7];
			dc = arr[8];
		}

		void BoidsModel::get_grid(){
			for (primatives::boid boid : boids){
				boid.ix = floor(((boid.p.x+cube_width)/rc));
				boid.iy = floor(((boid.p.y+cube_width)/rc));
				boid.iz = floor(((boid.p.z+cube_width)/rc));
				grid[boid.ix][boid.iy][boid.iz].push_back(boid);
			}
		}

		int get_ind(int i){
			if (i==0){
				return -1;
			} else if (i==1) {
				return 0;
			} else if (i==2) {
				return 1;
			}
		}

		vector<vector<primatives::boid>> BoidsModel::get_neighbourhood(unordered_map<int, unordered_map<int, unordered_map<int, vector<primatives::boid>>>> grid, primatives::boid boid){
			// cout << "boid location: ["<<boid.ix<<", "<<boid.iy<<", "<<boid.iz<<"]"<<std::endl;
			vector<vector<primatives::boid>> neighbourhood;

			for (int x = 0; x<3; x++){
				for (int y = 0; y<3; y++){
					for (int z = 0; z<3; z++){
						int ix = boid.ix + get_ind(x);
						int iy = boid.iy + get_ind(y);
						int iz = boid.iz + get_ind(z);
						if ((boid.ix==ix&&boid.iy==iy&&boid.iz==iz) || ix<0 || iy<0 || iz<0 || ix>gridmax || iy>gridmax || iz>gridmax){
							continue;
						} else {
							// cout << "n: ["<<ix<<", "<<iy<<", "<<iz<<"]"<<std::endl;
							neighbourhood.push_back(grid[ix][iy][iz]);
						}
					}
				}
			}
			return neighbourhood;
		}

		void BoidsModel::step(float dt) {
			gridmax = floor((cube_width+cube_width)/rc);
			for (primatives::boid& boid : boids){
				// cout<<"b_id: "<<boid.id<<std::endl;
				float ix = floor(((boid.p.x+cube_width)/rc));
				float iy = floor(((boid.p.y+cube_width)/rc));
				float iz = floor(((boid.p.z+cube_width)/rc));
		

				boid.f = {0,0,0};
				boid.neighbourhood.clear();
				boid.neighbourhood = get_neighbourhood(grid, boid);

				for (primatives::boid boid_i : grid[boid.ix][boid.iy][boid.iz]){
					if (boid.id!=boid_i.id){
						// cout<<"b_id: "<<boid.id<<std::endl;
						glm::vec3 p_ij = boid.p - boid_i.p;
						float d = glm::length(p_ij);
						boid.separation_force(ks, boid, p_ij, d);
						boid.alignment_force(ka, boid);
					}
				}

				for (vector<primatives::boid> n : boid.neighbourhood){
					for (primatives::boid boid_i : n){
						// cout<<"bi_id: "<<boid.id<<std::endl;
						boid.cohesion_force(kc, boid);
					}
				}
					
				// for (primatives::boid& boid_j : boids){
				// 	if (boid.id!=boid_j.id){
				// 		glm::vec3 p_ij = boid.p - boid_j.p;
				// 		float d = glm::length(p_ij);
				// 		float alpha = glm::dot(glm::normalize(p_ij), glm::normalize(boid.v));
				// 		if ((d<rs) && (alpha>cos(glm::radians(ds)))){
				// 			boid.separation_force(ks, boid_j, p_ij, d);
				// 		} else if ((d<ra) && (alpha>cos(glm::radians(da)))){
				// 			boid.alignment_force(ka, boid_j);
				// 		} else if ((d<rc) && (alpha>cos(glm::radians(dc)))){
				// 			boid.cohesion_force(kc, boid_j);
				// 		}
				// 	}
				// }
			}
			for (primatives::boid& boid : boids){
				boid.calc_avoidance(planes);
				boid.integrate(dt);
				for (float p_i : {boid.p.x, boid.p.y, boid.p.z}){
					if (p_i > 20 || p_i<-20){
						// cout << "id: " << boid.id << " p: [ " << boid.p.x << ", " << boid.p.y << ", "<< boid.p.z << ",]" << endl;
						break;
					}
				}
			}
		}

		void BoidsModel::render(const ModelViewContext& view) {
			//Add Mass render
			for (primatives::boid& boid : boids) {
				boid.f += g;
				glm::vec3 T = glm::normalize(boid.v);
				glm::vec3 N = glm::normalize(boid.f - (glm::dot(boid.f, T)*T));
				glm::vec3 B = glm::normalize(glm::cross(N,T));
				glm::mat4 M;
				M[0] = glm::vec4(B, 0.f);
				M[1] = glm::vec4(N, 0.f);
				M[2] = glm::vec4(T, 0.f);
				M[3] = glm::vec4(boid.p, 1.f);

				givr::addInstance(boid_render, M);
			}
			
			//Render
			givr::style::draw(boid_render, view);
			givr::style::draw(wall_render, view);
		}
	} // namespace models
} // namespace simulation