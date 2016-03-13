#include <etk/etk.h>
#include <fstream>

using namespace std;
using namespace etk;


int main()
{
	ofstream out;
	out.open("dump");
	
	
	const float speed = 20;
	const float dt = 0.02;
	const Vector<3> ang_vel(0.0, 0.1, 0.0);
	const Vector<3> gravity(0, 0, 9.8);
	
	Vector<3> velocity;
	Vector<3> acceleration;
	
	float heading = 0;
	float pitch = 0;
	float roll = 0;
	
	Quaternion q;
	
	Vector<3> last_world_velocity(speed, 0, 0);
	Vector<3> last_body_velocity(speed, 0, 0);
	
	for(int i = 0; i < 400; i++)
	{
		StaticString<100> ss;
		
		//print out the actual orientation, in euler angles
		out << "Heading: " << heading << " Pitch: " << pitch << " Roll: " << roll << endl;
		
		//the body velocity is speed along the x axis, plus some acceleration
		Vector<3> body_velocity(speed+(i/100.0), 0, 0);
		//rotate body velocity onto the world frame of reference
		auto world_velocity = q.rotateVector(body_velocity);
		
		//differentiate world velocity to find world acceleration
		auto world_acceleration = (world_velocity - last_world_velocity)/dt;
		last_world_velocity = world_velocity;
		
		//add gravity (9.8m/s/s UP)
		world_acceleration = world_acceleration + gravity;
		
		//rotate total world acceleration back to the body frame of reference
		//body_acceleration is what the accelerometers would sense
		auto body_acceleration = q.conjugate().rotateVector(world_acceleration);
		
		//print out body acceleration to log file
		ss.clear();
		ss += body_acceleration;
		out << "Body acceleration: " << ss.c_str() << endl;
		
		//pitch felt is the pitch angle as felt by the pilot (body frame)
		auto pitch_felt = atan2(-body_acceleration.x(), sqrt(body_acceleration.y()*body_acceleration.y() + body_acceleration.z()*body_acceleration.z()));
		out << "Pitch felt: " << radians_to_degrees(pitch_felt);
		
		//roll felt is the bank angle as detected by the pilot
		auto roll_felt = atan2(body_acceleration.y(), body_acceleration.z());
		out << " Roll felt: " << radians_to_degrees(roll_felt) << endl;
		ss.clear();
		
		
		//now calculate centrifugal force
		//angular velocity X body_velocity
		auto cf = ang_vel.cross(body_velocity);
		
		ss += cf;
		out << "Correction: " << ss.c_str() << endl;
		
		//subtract centrifugal force from body_acceleration
		body_acceleration = body_acceleration - cf;
		
		//differentiate body velocity to find linear acceleration
		auto linear_accel = (body_velocity - last_body_velocity)/dt;
		last_body_velocity = body_velocity;
		ss.clear();
		ss += linear_accel;
		out << "Linear accel: " <<  ss.c_str()<< endl;
		
		//remove linear acceleration from body acceleration
		body_acceleration = body_acceleration - linear_accel;
		
		//body acceleration should be a vector that truly points downwards
		//and has a magnitude of approximately 9.8m/s/s
		
		//print pitch and bank angle felt after corrections
		pitch_felt = atan2(-body_acceleration.x(), sqrt(body_acceleration.y()*body_acceleration.y() + body_acceleration.z()*body_acceleration.z()));
		out << "Corrected: " << radians_to_degrees(pitch_felt);
		
		roll_felt = atan2(body_acceleration.y(), body_acceleration.z());
		out << " " << radians_to_degrees(roll_felt) << endl;
		
		
		//integrate angular velocity to update q
		Quaternion qw;
		qw.fromAngularVelocity(ang_vel, dt);
		
		q = q*qw;
		
		//get euler angles from q
		auto euler = q.toEuler();
		euler.toDegrees();
		pitch = euler.y();
		roll = euler.z();
		heading = euler.x();
		
		out << endl << endl;
		
	}
	out.close();
	
}





