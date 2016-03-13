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
		out << "Heading: " << heading << " Pitch: " << pitch << " Roll: " << roll << endl;
		
		Vector<3> body_velocity(speed+(i/100.0), 0, 0);
		auto world_velocity = q.rotateVector(body_velocity);
		
		auto world_acceleration = (world_velocity - last_world_velocity)/dt;
		last_world_velocity = world_velocity;
		world_acceleration = world_acceleration + gravity;
		
		auto body_acceleration = q.conjugate().rotateVector(world_acceleration);
		ss.clear();
		ss += body_acceleration;
		out << "Body acceleration: " << ss.c_str() << endl;
		
		auto pitch_felt = atan2(-body_acceleration.x(), sqrt(body_acceleration.y()*body_acceleration.y() + body_acceleration.z()*body_acceleration.z()));
		out << "Pitch felt: " << radians_to_degrees(pitch_felt);
		
		auto roll_felt = atan2(body_acceleration.y(), body_acceleration.z());
		out << " Roll felt: " << radians_to_degrees(roll_felt) << endl;
		ss.clear();
		
		
		auto av = ang_vel;
		auto cf = av.cross(body_velocity);
		
		ss += cf;
		out << "Correction: " << ss.c_str() << endl;
		
		body_acceleration = body_acceleration - cf;
		
		auto linear_accel = (body_velocity - last_body_velocity)/dt;
		last_body_velocity = body_velocity;
		ss.clear();
		ss += linear_accel;
		out << "Linear accel: " <<  ss.c_str()<< endl;
		
		body_acceleration = body_acceleration - linear_accel;
		
		pitch_felt = atan2(-body_acceleration.x(), sqrt(body_acceleration.y()*body_acceleration.y() + body_acceleration.z()*body_acceleration.z()));
		out << "Corrected: " << radians_to_degrees(pitch_felt);
		
		roll_felt = atan2(body_acceleration.y(), body_acceleration.z());
		out << " " << radians_to_degrees(roll_felt) << endl;
		
		Quaternion qw;
		qw.fromAngularVelocity(ang_vel, dt);
		
		q = q*qw;
		
		auto euler = q.toEuler();
		euler.toDegrees();
		pitch = euler.y();
		roll = euler.z();
		heading = euler.x();
		
		out << endl << endl;
		
	}
	out.close();
	
}





