#include "models.hpp"
#include <random>

#include <glm/glm.hpp>

namespace simulation {
	namespace primatives {
		//No functions for structs, yet...
	}// namespace primatives

	namespace models {
		//Unless changed in main, this will call only once before the window is created
		BoidsModel::BoidsModel()
			: boid_geometry(givr::geometry::Mesh(givr::geometry::Filename("models/dart.obj")))
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
			static std::random_device random_device;
			static std::mt19937 generator(random_device());
			static std::uniform_real_distribution<double> position_distribution(-cube_width, cube_width);
			static std::uniform_real_distribution<double> theta_distribution(-180., 180.);
			static std::uniform_real_distribution<double> phi_distribution(-90., 90.);
			static std::uniform_real_distribution<double> speed_distribution(5., 25.);

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
				boid.v.x = speed * std::cos(theta) * std::cos(phi);
				boid.v.y = speed * std::sin(phi);
				boid.v.z = speed * std::sin(theta) * std::cos(phi);
			}
			grid.boids = &boids;
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

		void BoidsModel::step(float dt) {
			if (!(grid.scale == rc)) {
				grid.grid_constants(rc, cube_width);
			}
			for (int i = 0; i < grid.buckets.size(); i++) {
				if (grid.buckets[i].size()>0)
					grid.buckets[i].clear();
			}
			grid.move_boids();
			for (primatives::boid& boid_i : boids){
				boid_i.f = {0,0,0};
				/*for (primatives::boid& boid_j : boids){
					if (boid_i.id!=boid_j.id){
						glm::vec3 p_ij = boid_i.p - boid_j.p;
						float d = glm::length(p_ij);
						float alpha = glm::dot(glm::normalize(p_ij), glm::normalize(boid_i.v));
						if ((d<rs) && (alpha>cos(glm::radians(ds)))){
							boid_i.separation_force(ks, boid_j, p_ij, d);
						} else if ((d<ra) && (alpha>cos(glm::radians(da)))){
							boid_i.alignment_force(ka, boid_j);
						} else if ((d<rc) && (alpha>cos(glm::radians(dc)))){
							boid_i.cohesion_force(kc, boid_j);
						}
					}
				}*/
			}
			for (primatives::boid& boid_i : boids){
				boid_i.calc_avoidance(planes);
				boid_i.integrate(dt);
				for (float p_i : {boid_i.p.x, boid_i.p.y, boid_i.p.z}){
					if (p_i > 20 || p_i<-20){
						std::cout << "id: " << boid_i.id << " p: [ " << boid_i.p.x << ", " << boid_i.p.y << ", "<< boid_i.p.z << ",]" << std::endl;
						break;
					}
				}
			}
			/*cout << "\n" << endl;*/
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
				// boid.f = glm::vec3(0.f);
			}
			
			//Render
			givr::style::draw(boid_render, view);
			givr::style::draw(wall_render, view);
		}
	} // namespace models
} // namespace simulation