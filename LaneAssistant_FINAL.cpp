//  _               _   _ ______             _____ _____ _____  _____ _______       _   _ _______ 
// | |        /\   | \ | |  ____|     /\    / ____/ ____|_   _|/ ____|__   __|/\   | \ | |__   __|
// | |       /  \  |  \| | |__       /  \  | (___| (___   | | | (___    | |  /  \  |  \| |  | |   
// | |      / /\ \ | . ` |  __|     / /\ \  \___ \\___ \  | |  \___ \   | | / /\ \ | . ` |  | |   
// | |____ / ____ \| |\  | |____   / ____ \ ____) |___) |_| |_ ____) |  | |/ ____ \| |\  |  | |   
// |______/_/    \_\_| \_|______| /_/    \_\_____/_____/|_____|_____/   |_/_/    \_\_| \_|  |_|   

//GoHomeNow
//Projekt Shuttel / C++ / TRONIS by TWT

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <communication/multi_socket.h>
#include <models/tronis/ImageFrame.h>
#include <grabber/opencv_tools.hpp>
#include <models/tronis/BoxData.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <map>
#include <math.h>
#include <limits>



using namespace std;
using namespace cv;



//Klasse zur Beschleunigungsregelung ->FERTIG
class ThrottleControl
{
private:
	int TC_enable_distance_control = 1;
	
	// Geschwindigkeisregler PI
	int TC_set_speed = 50; //Zielgeschwindigkeit
	//soft
	int TC_Kp_speed_soft = 4, TC_Kd_speed_soft = 0, TC_Ki_speed_soft = 136;
	double TC_speed_error_old;
	double TC_throttle_control_condition = 0;
	double TC_raw_throttle_control_condition = 0;
	
	//Abstandsregler PID
	int TC_vehicle_distance = 0;
	int TC_vehicle_distance_old = 0;
	//Komfortbereich
	int TC_Kp_distance_soft = 7, TC_Ki_distance_soft = 1, TC_Kd_distance_soft = 5;

	double TC_distance_error_old = 0;
	//Zielabstand
	int TC_target_distance = 20;
	double TC_distance_error_sum = 0;
	int TC_distance_timer = 0;
	const int TC_distance_timer_max = 5;
	bool TC_vehicle_detected = false;
	
	//Relativgeschwindigkeit
	//double TC_relativ_velo = 0;
	//double TC_vordermann_velo = 0;
public:
	//Einstellung ob Distanceregelung aktiv sein soll oder nur auf Speed (Default: Distance)
	ThrottleControl(bool enable_distance_control)
	{
		TC_enable_distance_control = enable_distance_control;
		TC_throttleControlTrackbar();
	}

	//hier wird Objekt Distance in Class übergebn 
	void TC_set_distance(double distance) 
	{
		TC_vehicle_distance = distance;
	}

	//hier wird Objekt bool ob detected oder nicht in Class übergeben 
	void TC_set_detected(double detected) 
	{
		TC_vehicle_detected = detected;
	}

	// Gibt den aktuellen Befehl für Tronis zurück
	double TC_TronisCommand() 
	{
		return TC_throttle_control_condition;
	}

	// Funktion zur Skalierung in den Bereich [-1, 1]
	double scaleToRange(double value, double min, double max) {
		double absMax = std::max(std::abs(min), std::abs(max));
		if (std::abs(value) > absMax) {
			value = (value / std::abs(value)) * absMax;
		}
		return value;
	}

	//Master des Reglers welcher entwender auf Geschwindigkeit oder Abstand regelt 
	void TC_control(double TC_ego_velo_, double TC_curvature_) 
	{
		double TC_ego_velo = TC_ego_velo_ * 0.036;
		double TC_curvature = TC_curvature_;
		//Case Distanzregelung aktiv
		if (TC_enable_distance_control)
		{
			TC_controlDistance(TC_ego_velo, TC_curvature);
		}
		//Case Speedregelung aktiv
		if (!TC_enable_distance_control)
		{
			TC_controlSpeed(TC_ego_velo, TC_curvature);
		}
	}

	//PID Regler des Geschwindigkeitsregles
	void TC_controlSpeed(double TC_ego_velo_, double  TC_curvature_)
	{
		double TC_ego_velo = TC_ego_velo_;
		double TC_curvature = TC_curvature_;
		double TC_speed_error_sum;
		double TC_speed_error = (double)TC_set_speed - TC_ego_velo + 17; // Berechnet den Error welcher zwischen ist und soll Geschwindigkeit liegt 
		double deltaTime = 0.001;
		double TC_speed_error_difference = (TC_speed_error - TC_speed_error_old) / deltaTime; // Berechnung der Ableitung siehe SteeringController
		TC_speed_error_old = TC_speed_error;
		TC_speed_error_sum += TC_speed_error * deltaTime;
		cout << "Speed aktiv" << endl;
		//PID Regler für Abstandsregelung SOFT
		TC_raw_throttle_control_condition = (double)TC_Kp_speed_soft / 100 * TC_speed_error + (double)TC_Kd_speed_soft / 1000 * TC_speed_error_difference + (double)TC_Ki_speed_soft / 10000 * TC_speed_error_sum;
		

		//Für enge Kurven nutzen -> Damit Geschwindigkeit reduziert wird 
		/*if (TC_curvature > 0.001)
		{
			cout << "sehr enge Kurve -> reduziere geschwindigkeit " << endl;
			TC_raw_throttle_control_condition = TC_raw_throttle_control_condition/5;

		}*/


		// Proportionale Skalierung des Wertes in den Bereich [-1, 1]
		TC_throttle_control_condition = scaleToRange(TC_raw_throttle_control_condition, -1.0, 1.0);
		
		//Grafische Ausgabe von TC_vehicle_distance
		cout << "\033[2J\033[H"; // Clear Terminal 
		cout << "Graphische Ausgabe (letzte Werte):\n";
		for (int i = 0; i < TC_ego_velo / 2; i++)
		{
			cout << "=";
		}
		cout << "| " << TC_ego_velo << "km/h\n";
		
		
		
		
		//Verhindert rückwärtsfahren
		if (TC_set_speed == 0 && (TC_ego_velo < 5))
		{
			TC_throttle_control_condition = 0;
		}
		//soll ruckeln verhindern
		if (TC_set_speed < 10 && (TC_throttle_control_condition < 0))
		{
			TC_throttle_control_condition = 0;
		}
		
	}

	//PD Regler des Distanzreglers
	void TC_controlDistance(double TC_ego_velo_, double TC_curvature_)
	{
		double TC_ego_velo = TC_ego_velo_;
		double TC_curvature = TC_curvature_;


		////Festlegung des gesetzlichen Mindestabstand
		//TC_target_distance = TC_ego_velo / 2;
		//if (TC_ego_velo < 40)
		//{
		//	TC_target_distance = 20;
		//}
		//Abfrage ob Geschwindigkeits oder Abstandsregelung

		 // Wenn Distanz größer als 90m wird auf Geschwindigkeit geregelt
		if (TC_vehicle_distance > 90 || (TC_vehicle_detected == false))  
		{
			TC_controlSpeed(TC_ego_velo, TC_curvature);
		}


		//Ansonsten auf Abstand 
		if (TC_vehicle_distance < 90 && (TC_vehicle_detected == true))
		{
			cout << "Distanz aktiv " << endl;
			double TC_distance_error = TC_vehicle_distance - (double)(TC_target_distance);  // Berechnet den Error welcher zwischen ist und soll Distance liegt . -3 resultiert aus Reglertuning dient als y verischiebung 
			double TC_distance_error_soft = TC_vehicle_distance - (double)(TC_target_distance - 8);
			double deltaTime = 0.01;
			double TC_distance_Error_difference = (TC_distance_error - TC_distance_error_old) / deltaTime;
			TC_distance_error_old = TC_distance_error;
			
			
			//Summe des Errors für I-Glied
			TC_distance_error_sum += TC_distance_error * deltaTime;
			
			// Begrenzung der Fehler-Summe, um Integrator-Sättigung zu vermeiden
			double max_integral = 100.0; // Maximalwert für die Integratorsumme

			if (TC_distance_error_sum > max_integral)
			{
				TC_distance_error_sum = max_integral;
			}
			if (TC_distance_error_sum < -max_integral)
			{
				TC_distance_error_sum = -max_integral;
			}
				
			//Brechnung des Notwendigen Bremsweg abhängig der Geschwindigkeit (siehe ADAC)
			double Bremsweg = ((TC_ego_velo / 10)*(TC_ego_velo / 10)) / 2;

			// Auswahl des Reglers: Komfort oder Sicherheit
			if ((TC_vehicle_distance < TC_target_distance - 5) || (TC_vehicle_distance <= Bremsweg))
			{

				//cout << "Distanz aktiv SICHERHEITSZONE " << endl;
				//Agressiver Regler für Sicherheitszone
				TC_throttle_control_condition = -1;
			}

			//PID Regler für Abstandsregelung Konmfort
			else
			{
				// Komfortabler Regler
				//cout << " Distanz aktiv KOMFORTBETRIEB " << endl;
				TC_throttle_control_condition = (double)TC_Kp_distance_soft / 100 * TC_distance_error_soft + (double)TC_Ki_distance_soft / 100000 * TC_distance_error_sum + (double)TC_Kd_distance_soft / 10 * TC_distance_Error_difference;
			}


			//Grafische Ausgabe von TC_vehicle_distance
			cout << "\033[2J\033[H"; // Clear Terminal 
			cout << "Graphische Ausgabe (letzte Werte):\n";
			for (int i = 0; i < TC_vehicle_distance / 2; i++)
			{
				cout << "=";
			}
			cout << "| " << TC_vehicle_distance << "m\n";



			//Verhindert rückwärtsfahren
			if (TC_ego_velo < 5 && TC_throttle_control_condition < 0)
			{
				TC_throttle_control_condition = 0;
			}
			//das auch
			if (TC_set_speed == 0 && (TC_ego_velo < 5))
			{
				TC_throttle_control_condition = 0;
			}
						
			//Verdindert das während Distance Fahrt über Set Speed beschleunigt wird
			if (TC_ego_velo > TC_set_speed)
			{
				TC_throttle_control_condition = 0;
			}


			//Skalierung auf [-1,1]
			if (TC_throttle_control_condition > 1)
			{
				TC_throttle_control_condition = 1;
			}
				
			if (TC_throttle_control_condition < -1)
			{
				TC_throttle_control_condition = -1;
			}
				

		}
	}
	void TC_throttleControlTrackbar()
	{
		// Erzeuge Fenster mit Trackbars zur manuellen Anpassung
		namedWindow("Throttle Control", (600, 600));
		createTrackbar("enable", "Throttle Control", &TC_enable_distance_control, 1);
		createTrackbar("Soll V ", "Throttle Control", &TC_set_speed, 100); // Maximalwert von 200
		//createTrackbar("Kp_s_soft", "Throttle Control", &TC_Kp_speed_soft, 100);  // Maximalwert von 200
		//createTrackbar("Kd_s_soft", "Throttle Control", &TC_Kd_speed_soft, 500);  // Maximalwert von 200
		//createTrackbar("Ki_s_soft", "Throttle Control", &TC_Ki_speed_soft, 500);  // Maximalwert von 200
		createTrackbar("Ziel Abstand (Soll)", "Throttle Control", &TC_target_distance, 70);  // Maximalwert von 200
		//createTrackbar("Kp_distance", "Throttle Control", &TC_Kp_distance_soft, 200);  // Maximalwert von 200
		//createTrackbar("Ki_distance", "Throttle Control", &TC_Ki_distance_soft, 200);  // Maximalwert von 200
		//createTrackbar("Kd_distance", "Throttle Control", &TC_Kd_distance_soft, 200);  // Maximalwert von 200

	}
};

// Klasse zur Lenkungsregelung → FERTIG
class SteeringControl
{
private:
	double SC_steering_control_condition_raw = 0;
	double SC_steering_control_condition = 0;	// Ausgangsgröße des Reglers [-1,0,1] wird an Tronis übergeben 
	double SC_input_value = 0;					// Eingangsgröße Regler hier Delta der Midpoints
	int SC_Kp = 35;								// Proportionalitätsfaktor, der die Stellgröße proportional zur Abweichung macht. old 35
	int SC_Kd = 27;								// Differentialfaktor – reagiert auf die Änderungsrate der Abweichung und sorgt für eine dämpfende Wirkung. old 27
	double SC_steering_error_old = 0;
	double SC_steering_error_filtered = 0;
	double SC_steering_error_sum = 0;

	// Parameter für die Erkennung von Überschwingungen
	double SC_overshoot_threshold = 0.1; // Abweichungsschwelle zur Erkennung eines Überschwingens
	int SC_overshoot_counter = 0;        // Zähler für die Überschwingungen
	int SC_overshoot_limit = 3;          // Anzahl der Überschwingungen, bevor eine Anpassung erfolgt
	double SC_scaling_factor = 0.9;      // Faktor, um Kp und Kd bei Überschwingen zu reduzieren

	// Zeitmanagement zur Entprellung
	int64 SC_last_switch_time = 0;       // Zeitpunkt des letzten Vorzeichenwechsels
	double SC_debounce_time_ms = 100;    // Minimale Zeit (in ms) zwischen zwei Vorzeichenwechseln (Entprellzeit)
	double SC_control_scale_factor = 0;	 // Faktor welcher Lenkung skaliert

public:
	// Konstruktor erzeugt Slider
	SteeringControl()
	{
		SC_steeringControlTrackbar();
	}

	// Gibt den aktuellen Befehl für Tronis zurück
	double SC_TronisCommand() 
	{
		return SC_steering_control_condition;
	}

	// Funktion zur Skalierung in den Bereich [-1, 1]
	double scaleToRange(double value, double min, double max) {
		double absMax = std::max(std::abs(min), std::abs(max));
		if (std::abs(value) > absMax)
		{
			value = (value / std::abs(value)) * absMax;
		}
		return value;
	}
	// Hauptreglerfunktion
	double SC_Control(double SC_input_value, double SC_input_value2) 
	{
		// Berechne Fehler und Ableitung
		double SC_ego_velo = SC_input_value2 * 0.036;
		double SC_steering_error = SC_input_value; //- 2; // Kickstart des Reglers 
		double SC_steering_error_difference = SC_steering_error - SC_steering_error_old; // Ableitung durch Differenzbildung
		SC_steering_error_old = SC_steering_error; // Setze aktuellen Fehler als alten Fehler
		
		// Dynamische Anpassung von Kp und Kd basierend auf der Geschwindigkeit
		if (SC_ego_velo > 60.0) {
			SC_Kp = 35 - ((SC_ego_velo - 60) / 60.0) * 20;  // Reduziert Kp von 35 auf 15 bei 100 km/h
			SC_Kd = 27 + ((SC_ego_velo - 60) / 60.0) * 20;  // Erhöht Kd von 27 auf 47 bei 100 km/h
		}
		else 
		{
			SC_Kp = 35;  // Standard Kp bei 60 km/h 35
			SC_Kd = 27;  // Standard Kd bei 60 km/h  27
		}

		//Anpassung des Skalierungsfaktors abhängig von der Geschwindigkeit 
		if (SC_ego_velo < 50)
		{
			SC_control_scale_factor = 2.2;
		}
		if (50 <= SC_ego_velo)
		{
			SC_control_scale_factor = (1 / 2 * (SC_ego_velo - 50)) + 2.2;
		}

		// Regelungsgleichung
		SC_steering_control_condition_raw = double(SC_Kp) / 10000 * SC_steering_error + double(SC_Kd) / 10000 * SC_steering_error_difference;
		//Wert Skalierung
		SC_steering_control_condition_raw = SC_steering_control_condition_raw * SC_control_scale_factor;
		//Wert Normierung
		SC_steering_control_condition = scaleToRange(SC_steering_control_condition_raw, -1, 1);
		return SC_steering_control_condition;
	}
	void SC_steeringControlTrackbar()
	{
		// Erzeuge Fenster mit Trackbars zur manuellen Anpassung
		//namedWindow("Steering Control", (600, 400));
		//createTrackbar("Kp ", "Steering Control", &SC_Kp, 200); // Maximalwert von 200
		//createTrackbar("Kd", "Steering Control", &SC_Kd, 200);  // Maximalwert von 200
	}
};

// Klasse zur Fahrbahnmarkierungserkennung -> FERTIG
class LaneDetection
{
private:
	// Variablen Definition
	Mat InputPic;  // Eingabe Bild
	
				   //ROI Def
	int LD_x_val_roi = 0;
	int LD_y_val_roi = 260;
	int LD_width_roi = 720;
	int LD_height_roi = 100;
	
	//Bildverarbeitungs variablen
	// HSV values für weiß und gelb
	//########Weiß#########
	// Farbwert 
	int LD_hue_min_white = 93;
	int LD_hue_max_white = 187;
	// Sättigung
	int LD_sat_min_white = 0;
	int LD_sat_max_white = 150;
	// Hellwert
	int LD_val_min_white = 150;
	int LD_val_max_white = 255;
	//#######Gelb###########
	//Farbwert
	int LD_hue_min_yellow = 20;
	int LD_hue_max_yellow = 40;
	// Sättigung 
	int LD_sat_min_yellow = 150;
	int LD_sat_max_yellow = 255;
	// Hellwert 
	int LD_val_min_yellow = 150;
	int LD_val_max_yellow = 255;
	//######################
	// Canny Variabeln 
	int LD_canny_lower_threshold = 31;
	int LD_canny_higher_threshold = 180;

	// Magnitut Level
	int LD_magnilvl = 164;
	
	// Variable der Polynomordnung
	int LD_poly_order = 2;
	int LD_slope_thresh = 20;
	Point LD_left_lane_poly[2];   // same for poly fit
	Point LD_right_lane_poly[2];  // same for poly fit
	
	//Konturfinder values
	int LD_min_kontour_size = 50; //mindest Fläche einer Kontur welche Berücksichtigt werden soll 
	int LD_max_kontour_size = 10000;//maximal Fläche einer Kontur welche Berücksichtigt werden soll → Bei Zebrastreifen wichtig 

	//Baustellenmodus
	int LD_Baustellenmodus = 0;

	//Vector Dekleration
	vector<Point> LD_left_line;
	vector<Point> LD_right_line;
	//Fallback vector
	vector<Point> LD_left_line_fallback = {
	{112, 0}, {112, 4}, {111, 5}, {110, 5}, {110, 6}, {111, 7}, {111, 16}, {112, 17}, {112, 25}, {113, 26},
	{113, 28}, {112, 29}, {111, 29}, {111, 34}, {112, 35}, {112, 43}, {113, 44}, {113, 49}, {112, 50}, {112, 54},
	{113, 55}, {113, 63}, {114, 64}, {114, 68}, {113, 69}, {112, 68}, {112, 69}, {113, 70}, {113, 77}, {114, 78},
	{114, 84}, {113, 85}, {113, 91}, {114, 92}, {114, 99}, {115, 100}, {114, 101}, {113, 101}, {113, 103}, {114, 104},
	{114, 111}, {115, 112}, {115, 114}, {114, 115}, {113, 114}, {114, 115}, {114, 122}, {115, 123}, {115, 126}, {114, 127},
	{114, 129}, {115, 130}, {115, 136}, {116, 137}, {116, 138}, {115, 139}, {114, 139}, {115, 140}, {115, 146}, {116, 147},
	{116, 148}, {115, 149}, {115, 154}, {116, 155}, {116, 158}, {115, 159}, {115, 160}, {116, 161}, {116, 172}, {117, 173},
	{117, 176}, {116, 177}, {116, 179}, {117, 180}, {117, 190}, {118, 191}, {117, 192}, {117, 194}, {118, 195}, {118, 198},
	{117, 199}, {117, 201}, {118, 202}, {118, 204}, {117, 205}, {117, 206}, {118, 207}, {118, 224}, {119, 225}, {119, 226},
	{118, 227}, {119, 228}, {119, 231}, {118, 232}, {119, 233}, {119, 246}, {120, 247}, {120, 248}, {119, 249}, {120, 250},
	{120, 251}, {119, 252}, {120, 253}, {120, 269}, {121, 270}, {120, 271}, {120, 272}, {121, 273}, {120, 274}, {121, 275},
	{120, 276}, {121, 277}, {121, 292}, {122, 293}, {121, 294}, {122, 295}, {122, 296}, {121, 297}, {122, 298}, {122, 318},
	{123, 319}, {123, 336}, {124, 337}, {124, 338}, {123, 339}, {124, 340}, {124, 343}, {126, 343}, {126, 341}, {125, 340},
	{125, 329}, {124, 328}, {124, 327}, {125, 326}, {125, 323}, {124, 322}, {124, 320}, {125, 319}, {124, 318}, {124, 293},
	{123, 292}, {124, 291}, {124, 289}, {123, 288}, {124, 287}, {124, 285}, {123, 284}, {124, 283}, {123, 282}, {123, 281},
	{124, 280}, {123, 279}, {124, 278}, {123, 277}, {123, 253}, {122, 252}, {123, 251}, {123, 250}, {122, 249}, {123, 248},
	{123, 246}, {122, 245}, {123, 244}, {123, 243}, {122, 242}, {122, 241}, {123, 240}, {122, 239}, {122, 237}, {123, 236},
	{123, 235}, {122, 234}, {122, 227}, {123, 226}, {122, 225}, {122, 206}, {121, 205}, {122, 204}, {122, 201}, {121, 200},
	{121, 199}, {122, 198}, {122, 194}, {121, 193}, {121, 192}, {122, 191}, {122, 189}, {121, 188}, {121, 185}, {122, 184},
	{122, 183}, {121, 182}, {121, 177}, {122, 176}, {122, 175}, {121, 174}, {121, 160}, {122, 159}, {122, 158}, {121, 157},
	{121, 143}, {120, 142}, {120, 139}, {121, 138}, {121, 131}, {120, 130}, {120, 128}, {121, 127}, {121, 118}, {120, 117},
	{120, 115}, {121, 114}, {122, 114}, {121, 113}, {121, 104}, {120, 103}, {120, 101}, {121, 100}, {121, 93}, {120, 92},
	{120, 86}, {121, 85}, {121, 76}, {120, 75}, {120, 69}, {121, 68}, {121, 64}, {120, 63}, {120, 53}, {119, 52},
	{119, 50}, {120, 49}, {121, 50}, {121, 44}, {120, 43}, {120, 32}, {119, 31}, {119, 30}, {120, 29}, {121, 29},
	{120, 28}, {120, 16}, {119, 15}, {119, 6}, {120, 5}, {120, 0}
	};
	vector<Point> LD_right_line_fallback = {
	{182, 0}, {182, 6}, {181, 7}, {181, 17}, {182, 17}, {183, 18}, {183, 26}, {182, 27}, {182, 38}, {181, 39},
	{181, 40}, {182, 41}, {182, 52}, {181, 53}, {181, 60}, {182, 61}, {182, 65}, {181, 66}, {181, 76}, {180, 77},
	{180, 78}, {181, 78}, {182, 79}, {182, 83}, {181, 84}, {181, 93}, {180, 94}, {181, 95}, {181, 104}, {180, 105},
	{180, 109}, {181, 110}, {181, 118}, {180, 119}, {180, 122}, {181, 123}, {181, 127}, {180, 128}, {180, 134}, {181, 135},
	{180, 136}, {180, 144}, {179, 145}, {181, 147}, {180, 148}, {180, 156}, {181, 157}, {180, 158}, {180, 172}, {179, 173},
	{179, 174}, {180, 175}, {180, 178}, {179, 179}, {179, 182}, {180, 183}, {180, 187}, {179, 188}, {179, 190}, {180, 191},
	{180, 192}, {179, 193}, {179, 203}, {180, 204}, {179, 205}, {179, 220}, {178, 221}, {179, 222}, {179, 225}, {178, 226},
	{179, 227}, {179, 229}, {178, 230}, {178, 231}, {179, 232}, {179, 234}, {178, 235}, {179, 236}, {179, 238}, {178, 239},
	{178, 240}, {179, 241}, {178, 242}, {178, 282}, {177, 283}, {177, 307}, {176, 308}, {176, 309}, {177, 310}, {177, 311},
	{176, 312}, {176, 314}, {177, 315}, {176, 316}, {176, 339}, {175, 340}, {175, 342}, {174, 343}, {176, 343}, {177, 342},
	{177, 324}, {178, 323}, {178, 308}, {179, 307}, {179, 306}, {178, 305}, {179, 304}, {179, 291}, {180, 290}, {179, 289},
	{180, 288}, {179, 287}, {180, 286}, {180, 268}, {181, 267}, {181, 266}, {180, 265}, {180, 264}, {181, 263}, {180, 262},
	{181, 261}, {181, 260}, {180, 259}, {181, 258}, {181, 246}, {182, 245}, {181, 244}, {181, 243}, {182, 242}, {182, 241},
	{181, 240}, {181, 239}, {182, 238}, {182, 228}, {183, 227}, {182, 226}, {182, 225}, {183, 224}, {183, 222}, {182, 221},
	{182, 220}, {183, 219}, {183, 217}, {182, 216}, {183, 215}, {183, 207}, {184, 206}, {184, 205}, {183, 204}, {183, 201},
	{184, 200}, {184, 198}, {183, 197}, {183, 196}, {184, 195}, {184, 191}, {183, 190}, {184, 189}, {184, 179}, {185, 178},
	{185, 176}, {184, 175}, {184, 171}, {185, 170}, {185, 167}, {184, 166}, {185, 165}, {185, 159}, {186, 158}, {185, 158},
	{184, 157}, {185, 156}, {185, 150}, {186, 149}, {186, 148}, {185, 147}, {185, 141}, {186, 140}, {186, 137}, {185, 136},
	{185, 134}, {186, 133}, {186, 127}, {187, 126}, {187, 125}, {186, 124}, {186, 120}, {187, 119}, {187, 111}, {186, 110},
	{186, 108}, {187, 107}, {187, 100}, {188, 99}, {188, 97}, {187, 96}, {187, 92}, {188, 91}, {188, 83}, {189, 82},
	{189, 81}, {188, 81}, {187, 80}, {187, 78}, {188, 77}, {188, 69}, {189, 68}, {189, 63}, {188, 62}, {188, 54},
	{189, 53}, {189, 45}, {190, 44}, {190, 43}, {189, 42}, {189, 34}, {190, 33}, {190, 24}, {191, 23}, {191, 21},
	{190, 20}, {190, 12}, {191, 11}, {191, 2}, {192, 1}, {192, 0}
	};
	///////////////////////////////////////// Falls keine Linien im ersten Moment erkannt werden nutze Fallback 
	vector<Point> LD_left_line_old = {
	{112, 0}, {112, 4}, {111, 5}, {110, 5}, {110, 6}, {111, 7}, {111, 16}, {112, 17}, {112, 25}, {113, 26},
	{113, 28}, {112, 29}, {111, 29}, {111, 34}, {112, 35}, {112, 43}, {113, 44}, {113, 49}, {112, 50}, {112, 54},
	{113, 55}, {113, 63}, {114, 64}, {114, 68}, {113, 69}, {112, 68}, {112, 69}, {113, 70}, {113, 77}, {114, 78},
	{114, 84}, {113, 85}, {113, 91}, {114, 92}, {114, 99}, {115, 100}, {114, 101}, {113, 101}, {113, 103}, {114, 104},
	{114, 111}, {115, 112}, {115, 114}, {114, 115}, {113, 114}, {114, 115}, {114, 122}, {115, 123}, {115, 126}, {114, 127},
	{114, 129}, {115, 130}, {115, 136}, {116, 137}, {116, 138}, {115, 139}, {114, 139}, {115, 140}, {115, 146}, {116, 147},
	{116, 148}, {115, 149}, {115, 154}, {116, 155}, {116, 158}, {115, 159}, {115, 160}, {116, 161}, {116, 172}, {117, 173},
	{117, 176}, {116, 177}, {116, 179}, {117, 180}, {117, 190}, {118, 191}, {117, 192}, {117, 194}, {118, 195}, {118, 198},
	{117, 199}, {117, 201}, {118, 202}, {118, 204}, {117, 205}, {117, 206}, {118, 207}, {118, 224}, {119, 225}, {119, 226},
	{118, 227}, {119, 228}, {119, 231}, {118, 232}, {119, 233}, {119, 246}, {120, 247}, {120, 248}, {119, 249}, {120, 250},
	{120, 251}, {119, 252}, {120, 253}, {120, 269}, {121, 270}, {120, 271}, {120, 272}, {121, 273}, {120, 274}, {121, 275},
	{120, 276}, {121, 277}, {121, 292}, {122, 293}, {121, 294}, {122, 295}, {122, 296}, {121, 297}, {122, 298}, {122, 318},
	{123, 319}, {123, 336}, {124, 337}, {124, 338}, {123, 339}, {124, 340}, {124, 343}, {126, 343}, {126, 341}, {125, 340},
	{125, 329}, {124, 328}, {124, 327}, {125, 326}, {125, 323}, {124, 322}, {124, 320}, {125, 319}, {124, 318}, {124, 293},
	{123, 292}, {124, 291}, {124, 289}, {123, 288}, {124, 287}, {124, 285}, {123, 284}, {124, 283}, {123, 282}, {123, 281},
	{124, 280}, {123, 279}, {124, 278}, {123, 277}, {123, 253}, {122, 252}, {123, 251}, {123, 250}, {122, 249}, {123, 248},
	{123, 246}, {122, 245}, {123, 244}, {123, 243}, {122, 242}, {122, 241}, {123, 240}, {122, 239}, {122, 237}, {123, 236},
	{123, 235}, {122, 234}, {122, 227}, {123, 226}, {122, 225}, {122, 206}, {121, 205}, {122, 204}, {122, 201}, {121, 200},
	{121, 199}, {122, 198}, {122, 194}, {121, 193}, {121, 192}, {122, 191}, {122, 189}, {121, 188}, {121, 185}, {122, 184},
	{122, 183}, {121, 182}, {121, 177}, {122, 176}, {122, 175}, {121, 174}, {121, 160}, {122, 159}, {122, 158}, {121, 157},
	{121, 143}, {120, 142}, {120, 139}, {121, 138}, {121, 131}, {120, 130}, {120, 128}, {121, 127}, {121, 118}, {120, 117},
	{120, 115}, {121, 114}, {122, 114}, {121, 113}, {121, 104}, {120, 103}, {120, 101}, {121, 100}, {121, 93}, {120, 92},
	{120, 86}, {121, 85}, {121, 76}, {120, 75}, {120, 69}, {121, 68}, {121, 64}, {120, 63}, {120, 53}, {119, 52},
	{119, 50}, {120, 49}, {121, 50}, {121, 44}, {120, 43}, {120, 32}, {119, 31}, {119, 30}, {120, 29}, {121, 29},
	{120, 28}, {120, 16}, {119, 15}, {119, 6}, {120, 5}, {120, 0}
	};
	vector<Point> LD_right_line_old = {
	{182, 0}, {182, 6}, {181, 7}, {181, 17}, {182, 17}, {183, 18}, {183, 26}, {182, 27}, {182, 38}, {181, 39},
	{181, 40}, {182, 41}, {182, 52}, {181, 53}, {181, 60}, {182, 61}, {182, 65}, {181, 66}, {181, 76}, {180, 77},
	{180, 78}, {181, 78}, {182, 79}, {182, 83}, {181, 84}, {181, 93}, {180, 94}, {181, 95}, {181, 104}, {180, 105},
	{180, 109}, {181, 110}, {181, 118}, {180, 119}, {180, 122}, {181, 123}, {181, 127}, {180, 128}, {180, 134}, {181, 135},
	{180, 136}, {180, 144}, {179, 145}, {181, 147}, {180, 148}, {180, 156}, {181, 157}, {180, 158}, {180, 172}, {179, 173},
	{179, 174}, {180, 175}, {180, 178}, {179, 179}, {179, 182}, {180, 183}, {180, 187}, {179, 188}, {179, 190}, {180, 191},
	{180, 192}, {179, 193}, {179, 203}, {180, 204}, {179, 205}, {179, 220}, {178, 221}, {179, 222}, {179, 225}, {178, 226},
	{179, 227}, {179, 229}, {178, 230}, {178, 231}, {179, 232}, {179, 234}, {178, 235}, {179, 236}, {179, 238}, {178, 239},
	{178, 240}, {179, 241}, {178, 242}, {178, 282}, {177, 283}, {177, 307}, {176, 308}, {176, 309}, {177, 310}, {177, 311},
	{176, 312}, {176, 314}, {177, 315}, {176, 316}, {176, 339}, {175, 340}, {175, 342}, {174, 343}, {176, 343}, {177, 342},
	{177, 324}, {178, 323}, {178, 308}, {179, 307}, {179, 306}, {178, 305}, {179, 304}, {179, 291}, {180, 290}, {179, 289},
	{180, 288}, {179, 287}, {180, 286}, {180, 268}, {181, 267}, {181, 266}, {180, 265}, {180, 264}, {181, 263}, {180, 262},
	{181, 261}, {181, 260}, {180, 259}, {181, 258}, {181, 246}, {182, 245}, {181, 244}, {181, 243}, {182, 242}, {182, 241},
	{181, 240}, {181, 239}, {182, 238}, {182, 228}, {183, 227}, {182, 226}, {182, 225}, {183, 224}, {183, 222}, {182, 221},
	{182, 220}, {183, 219}, {183, 217}, {182, 216}, {183, 215}, {183, 207}, {184, 206}, {184, 205}, {183, 204}, {183, 201},
	{184, 200}, {184, 198}, {183, 197}, {183, 196}, {184, 195}, {184, 191}, {183, 190}, {184, 189}, {184, 179}, {185, 178},
	{185, 176}, {184, 175}, {184, 171}, {185, 170}, {185, 167}, {184, 166}, {185, 165}, {185, 159}, {186, 158}, {185, 158},
	{184, 157}, {185, 156}, {185, 150}, {186, 149}, {186, 148}, {185, 147}, {185, 141}, {186, 140}, {186, 137}, {185, 136},
	{185, 134}, {186, 133}, {186, 127}, {187, 126}, {187, 125}, {186, 124}, {186, 120}, {187, 119}, {187, 111}, {186, 110},
	{186, 108}, {187, 107}, {187, 100}, {188, 99}, {188, 97}, {187, 96}, {187, 92}, {188, 91}, {188, 83}, {189, 82},
	{189, 81}, {188, 81}, {187, 80}, {187, 78}, {188, 77}, {188, 69}, {189, 68}, {189, 63}, {188, 62}, {188, 54},
	{189, 53}, {189, 45}, {190, 44}, {190, 43}, {189, 42}, {189, 34}, {190, 33}, {190, 24}, {191, 23}, {191, 21},
	{190, 20}, {190, 12}, {191, 11}, {191, 2}, {192, 1}, {192, 0}
	};
	
	
	//default untere Bildpunkte bestimmung via ChatGPT lol
	Point2f LD_left_line_bottom = { 130,343 };
	Point2f LD_right_line_bottom = { 174,345 };
	
	//Mittelpunktdefinition
	double LD_mid_point = 0;
	double LD_mid_point_old = 0;
	//Glättung middle point
	float smoothed_middle_point_y = 0.0f; // Initialisierung des geglätteten Werts
	float smoothing_factor = 0.9f;        // Glättungsfaktor (zwischen 0 und 1), kleinere Werte = stärkere Glättung
	
	//Polykoeffes definition
	Mat coeffsLeft;
	Mat coeffsRight;
	Mat coeffsLeft_same;
	Mat coeffsRight_same;
	Point2f single_line_bottom = 0;
	//Abfrage ob Linien gleich sind (Funktionsübergreifende Information)
	bool Flag = false;

protected:
	double curvature_;


public:
	// Erzeuge Silder
	LaneDetection()
	{
		slider();
	}
	// hier wäre noch Funktion sinnvoll welche zur Laufzeit Parameter manipulieren könnte
	void slider()
	{
		namedWindow("LaneDetection Silder", (1200, 600));  // Silder window um Werte zu manipulieren
		//createTrackbar("Low_canny", "LaneDetection Silder", &LD_canny_lower_threshold, 255);
		//createTrackbar("High_canny", "LaneDetection Silder", &LD_canny_higher_threshold, 255);
		//createTrackbar("Mag_lvl", "LaneDetection Silder", &LD_magnilvl, 255);  // value range 0 .. 255
		//createTrackbar("Min_Size", "LaneDetection Silder", &LD_min_kontour_size, 255);  // value range 0 .. 255
		//createTrackbar("Max_size", "LaneDetection Silder", &LD_max_kontour_size, 10000);  // value range 0 .. 255
		createTrackbar("Baustellenmodus", "LaneDetection Silder", &LD_Baustellenmodus, 1);  // value range 0 .. 255
	}

	// gesamte Pipeline zur Detection von Linien
	double LaneFinderPipeline(Mat image_)  // input sollte das image sein
	{
		//Overview:
		//1. Input Image 
		//2. definieren der ROI (ideal Dreieck → um Fahrspuren zu inkludieren) 
		//3.Linien Detection und Auswahl + Baustellenmode
		//4. BEV Trafo in BirdsEYEview
		//5. Konturerkennung
		//6. Kontur wird zu Points Transformiert hier wird ebenfalls die Mittelpunktberechnung durchgeführt
		//7. Rücktrafo von BEV in Normales Bild
		//8. Mapping der Linien auf Orginal image


		// hier die Funktionen zur Lanedetection
		// 1. Input Image + Anzeigen des InputBildes
		InputPic = image_;
		//LD_showImage("Input Bild",InputPic);

		// 2. definieren der ROI (ideal Dreieck → um Fahrspuren zu inkludieren) + Anzeige
		Mat RoiPic = LD_ROI(InputPic, LD_x_val_roi, LD_y_val_roi, LD_width_roi, LD_height_roi);  // x=0, y=260, width=720, heigth=100: alles in Pixel
		//LD_showImage("ROI Bild", RoiPic);

		// 3. Linien Detection und Auswahl
		//Weiß
		Mat LinePic_White = LD_LineDetection(RoiPic, LD_canny_lower_threshold, LD_canny_higher_threshold, LD_hue_min_white, LD_hue_max_white, LD_sat_min_white, LD_sat_max_white, LD_val_min_white, LD_val_max_white, LD_magnilvl);
		//LD_showImage("Line Bild white", LinePic_White);
		//Gelb 
		Mat LinePic_Yellow = LD_LineDetection(RoiPic, LD_canny_lower_threshold, LD_canny_higher_threshold, LD_hue_min_yellow, LD_hue_max_yellow, LD_sat_min_yellow, LD_sat_max_yellow, LD_val_min_yellow, LD_val_max_yellow, LD_magnilvl);
		//LD_showImage("Line Bild_yellow", LinePic_Yellow);
		//Auswahl

		//3.1 Auswahl welche Linien verwendet werden sollen +  Baustellenmodus
		Mat LinePic = LD_LineDeciscion(LinePic_White, LinePic_Yellow); // Zusammenführung von weiß und gelben Linien 
		//LD_showImage("White&Yellow_image:", LinePic);

		// 4. BEV Trafo 
		Mat BEV_image = LD_Org2BEV(LinePic);
		//LD_showImage("BEV_image:", BEV_image);

		// 5. Konturerkennung → Funktion ist Krank mächtig Holy
		Mat imgLine = LD_ContourFinder(BEV_image);
		//LD_showImage("After Konturfinder:", imgLine);

		// 6. Kontur wird zu Points Transformiert hier wird ebenfalls die Mittelpunktberechnung durchgeführt
		Mat imgBothLines = LD_ContourToPoint_Midpoint(imgLine);
		//LD_showImage("Kontur2Points:", imgBothLines);

		//7. Rücktrafo von BEV in Normales Bild
		Mat LinenOrg = LD_BEV2Org(imgBothLines);
		//LD_showImage("Rücktrafo in ROI", LinenOrg);

		//8. Mapping der Linien auf Orginal image
		Mat output = LD_LinesOnOrg(LinenOrg, InputPic);
		LD_showImage("Finales Bild", output);
		return LD_mid_point;
	}

	//2 Function um Rectangle ROI zu erzeugen
	Mat LD_ROI(Mat img_, int x_, int y_, int width_, int height_)
	{
		Mat image = img_.clone();  // Klonen es Inputbildes → ermöglicht es das ursprüungliche Bild nicht zu verändern
		Rect roi(x_, y_, width_, height_);  // Erzeugung einer Rechteckigen ROI mit dem Startpunkt x/y und einer Höhe und Breite (Achtung x=0 und y=0 liegen im linken oberen Eck des Bildes → siehe BV)
		Mat image_roi = image(roi);  // erzeugt eine Teilansich des Eingangsbildes mit der roi
		return image_roi;
	}
	//3 Funktion welche Canny und HSV Trafo durchführt und beide & Verknüpft in 
	Mat LD_LineDetection(Mat RoiPic_, int canny_lower_threshold_, int canny_high_threshold_, int hue_min_, int hue_max_, int sat_min_, int sat_max_, int val_min_, int val_max_, int magnilvl_)
	{
		// In dieser Funktion wird die Liniendetektions durchgeführt
		//Overview:
		//1. Gamma-Korrketur -> Bildaufhellung
		//2. Histogramm-Ausgleich mit CLAHE ->  Kontrast zu verbessern
		//3. Grauß Filter -> zur Rauschreduzierung
		//4. CannyFilters -> Kantendetektion
		//5. HSV- Trafo -> Linien Extraktion anhand Farbe
		//6. Kombination Canny&HSV

		//Dekleration
		Mat RoiPic_lokal = RoiPic_.clone();
		Mat GrayPic_lokal;
		Mat gamma_corrected;
		Mat float_image;


		//Gamma-Korrektur zur Bildaufhellung in dunkeln Bereichen 
		RoiPic_lokal.convertTo(float_image, CV_32F, 1.0 / 255.0);	// Normierung des Bildes auf den Wertebereich [0,1]
		pow(float_image, 1.2, gamma_corrected);						// Anwendung der Gamma-Korrektur Gamma-Wert 1.2 -> Hier bedeutet 1.2, dass dunklere Bereiche etwas aufgehellt werden.
		gamma_corrected.convertTo(gamma_corrected, CV_8U, 255.0);	// Rückkonvertierung in CV_8U also Wertebereich [0,255]



		// Histogramm-Ausgleich mit CLAHE um den Kontrast zu verbessern.
		Mat lab_image;
		cvtColor(gamma_corrected, lab_image, cv::COLOR_BGR2Lab); // Umwandlung des Bildes in den Lab-Farbraum Lab-Kanäle (L, a, b)  L-Kanal enthält die Helligkeitsinformationen,  a- und b-Kanäle enthalten Farbinformationen.
		vector<cv::Mat> lab_channels;
		split(lab_image, lab_channels); // Kanäle (L, a, b) trennen  L (Luminanz), a (Grün-Rot), b (Blau-Gelb).
		Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8)); // Erstellung des CLAHE-Filters:  Kontrastlimit und Blockgröße
		clahe->apply(lab_channels[0], lab_channels[0]); // CLAHE wird nur auf den Luminanzkanal (Helligkeitskanal) angewendet -> Farbinformation erhalten nur Kontrast optimiert 
		merge(lab_channels, lab_image); // Kanäle wieder zusammenführen
		cvtColor(lab_image, gamma_corrected, cv::COLOR_Lab2BGR); // Zurück in BGR konvertieren
		cvtColor(gamma_corrected, GrayPic_lokal, COLOR_BGR2GRAY);  // Konvertierung in Grauwertbild


		////Grauß -> zur Rauschreduzierung
		Mat GaußPic_lokal;
		GaussianBlur(GrayPic_lokal, GaußPic_lokal, Size(3, 3), 3, 3);  // Anwendung des Gaußfilters

		// Anwendung des CannyFilters
		Mat cannyPic_lokal;
		Canny(GaußPic_lokal, cannyPic_lokal, canny_lower_threshold_, canny_high_threshold_);
		//imshow("Canny", cannyPic_lokal);


		// HSV
		Mat HSVPic_lokal, HSVPic_thresh_lokal, CombineCannyHSV_lokal;
		cvtColor(gamma_corrected, HSVPic_lokal, COLOR_BGR2HSV);		// Konvertierung in HSV Farbraum H = Hue (Farbton), S = Saturation (Sättigung), V = Value (Helligkeit)
		Scalar low(hue_min_, sat_min_, val_min_);					// Definition der Thresholds anhand Farbe extrahiert wird
		Scalar high(hue_max_, sat_max_, val_max_);
		inRange(HSVPic_lokal, low, high, HSVPic_thresh_lokal);		// Threshold (inRange) auf das HSV-Bild anwenden, um eine Maske zu erstellen
		//imshow("HSV", HSVPic_thresh_lokal);


		// Kombinieren von Canny und HSV
		Mat combined_binary = Mat::zeros(cannyPic_lokal.size(), CV_8UC1);	//Erstellen eines leeren Binärbildes
		Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));			//Erstellt eine 3x3 rechteckige Kernel - Matrix
		dilate(cannyPic_lokal, cannyPic_lokal, kernel);						//Dilate (Dilatation) macht Kanten breiter
		dilate(HSVPic_thresh_lokal, HSVPic_thresh_lokal, kernel);			//Dilate (Dilatation) macht Kanten breiter
		//Finales Bild
		combined_binary = (cannyPic_lokal & HSVPic_thresh_lokal);			//Kombiniert UND Vernknüpft alle Canny und HSV
		//imshow("KOMBI", combined_binary);
		return combined_binary;
	}

	//3.1 Unterscheideungsfunc weiße gelbe Linien 
	Mat LD_LineDeciscion(Mat linePic_White_, Mat linePic_Yellow)
	{
		//In dieser Funktion wird unterschieden welche Linien erkannt werden ACHTUNG RETURN IST EIN LINIEN BILD KEINE KONTUR
		//Overview:
		//1. Input mit weißen und gelben Bildern(linien)
		//2. Konturfinden und Flächenbestimmung weißer und gelber Konturfläche
		//3. Abfrage Baustellenmodus



		//1. Input mit weißen und gelben Bildern(linien)
		Mat Pic_white_lokal = linePic_White_.clone();
		Mat Pic_yellow_lokal = linePic_Yellow.clone();
		Mat Pic_combine;
		vector<std::vector<cv::Point>> contours_white;
		vector<cv::Vec4i> hierarchy_white;

		vector<std::vector<cv::Point>> contours_yellow;
		vector<cv::Vec4i> hierarchy_yellow;

		//2. Konturfinden und Flächenbestimmung weißer und gelber Konturfläche
		//Arena bestimmung weiße Flächen 
		findContours(Pic_white_lokal, contours_white, hierarchy_white, RETR_LIST, CHAIN_APPROX_NONE);
		// Gesamtfläche weiß berechnen
		double totalArea_white = 0.0;
		for (const auto & contours_white : contours_white) {
			totalArea_white += contourArea(contours_white);
		}
		//cout << "Gesamtfläche aller weißer Konturen: " << totalArea_white << " Pixel" << std::endl;

		//Arena bestimmung gelben  Flächen 
		findContours(Pic_yellow_lokal, contours_yellow, hierarchy_yellow, RETR_LIST, CHAIN_APPROX_NONE);
		// Gesamtfläche gelb berechnen
		double totalArea_yellow = 0.0;
		for (const auto & contours_yellow : contours_yellow) {
			totalArea_yellow += contourArea(contours_yellow);
		}
		//cout << "Gesamtfläche aller yellow Konturen: " << totalArea_yellow << " Pixel" << std::endl;

		//3. Abfrage Baustellenmodus
		//Baustellenmodus: hier sollen nur gelbe Linien beachtet werden 
		if (LD_Baustellenmodus == 1 )
		{
			if (totalArea_yellow > 100)
			{

				Pic_combine = Pic_yellow_lokal;
			}
			else
				//evtl auch beide combiniert
				//bitwise_or(Pic_white_lokal, Pic_yellow_lokal, Pic_combine);
				Pic_combine = Pic_white_lokal;
		}
		else
		{
			//weiße und Gelbe Linien werden erkannt und verwendet
			bitwise_or(Pic_white_lokal, Pic_yellow_lokal, Pic_combine);

		}

		//Return des Bildes mit relevanten Linien 
		return Pic_combine;
	}

	//4 Perspektiven Transformation in BEV
	Mat LD_Org2BEV(Mat combinePic_)
	{
		Mat BEV, BEV_inv;
		Mat imgTransformed2BEV;
		Mat imgCannyAndColor = combinePic_.clone();
		// perspective transform

		// Definition der Eingangspunkte
		vector<Point2f> input_points(4);
		input_points[0] = Point2f(261, 13);   // TL (Top Left) Points2f(x,y)
		input_points[1] = Point2f(423, 14);   // TR (Top Right)
		input_points[2] = Point2f(609, 102);  // BR (Bottom Right)
		input_points[3] = Point2f(147, 99);   // BL (Bottom Left)

		// Definition der Zielpunkte
		vector<Point2f> target_points(4);
		target_points[0] = Point2f(150 - 50, 400 - 302);  // TL
		target_points[1] = Point2f(150 + 30, 400 - 277);
		target_points[2] = Point2f(150 + 24, 400 - 55);
		target_points[3] = Point2f(150 - 20, 400 - 57);

		// Transformation input→target also von Normal → BEV
		BEV = getPerspectiveTransform(input_points, target_points);  
		Size warpedSize = Size(300, 400);  // legt Abmessung des Bildes nach der Trafo fest
		warpPerspective(imgCannyAndColor, imgTransformed2BEV, BEV, warpedSize);


		return imgTransformed2BEV;
	}

	//5 Konturfinder Funktion weil Hough Trafo rumspakt
	Mat LD_ContourFinder(Mat input_)
	{
		//In dieser Funktion werden die Gefundenen Konturen zugeordnet und ein Vektor mit Punkten für LeftLine und RightLine gebildet
		//Overview:
		//1. Finde Konturen mittels open cv Function
		//2. Abfrage ob überhaupt Konturen erkannt wurden -> wenn nicht nutze alte Kontur bzw default werte für alte Kontur
		//3. Schleife durch alle Konturen- hier wird auf Mindestgröße geprüft und dann ein Zuordnung der der Kontur zur LeftLine oder RightLine gemacht
		//4. Abfrage wenn nur eine Linie erkannt wird -Überprüfung welche Linie fehlt und dann kopie der vorhandnen Linien um fehlende Linie zu interpolieren
		//5. Verhinderung von Sprüungen der Linien nur FallbackSprung wird zugelassen
		//6. Check ob Linien zu nahe oder zu weit aneinander nicht verwerden werden Linien neu inizialisiert	 
		//7. Sichere Werte 



		//Dekleration 
		Mat imgLines_lokal;
		vector<vector<Point>> LD_contours;			//Eine Liste von gefundenen Konturen. Jede Kontur ist ein Numpy-Array von (x, y)-Koordinatenpunkten, die die Kontur repräsentieren.
		vector<Vec4i> LD_hierarchy;					//Ein Numpy-Array, das Informationen über die hierarchische Beziehung zwischen den Konturen enthält: [next, previous, first_child, parent]
		float LD_left_line_distant_temp = -1000;
		float LD_right_line_distant_temp = -1000;



		//1. Finde Konturen mittels open cv Function
		findContours(input_, LD_contours, LD_hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
		cvtColor(input_, input_, COLOR_GRAY2BGR);						//wandelt in Farbbild um um Farben zu nutzen 



		//1.1 Rückhgabe Bild definieren  → Nur Notwendig das Funtkion rückgabewert hat 
		imgLines_lokal = input_.clone();
		rectangle(imgLines_lokal, Point(0, 0), Point(300, 400), Scalar(0, 0, 0), FILLED); //Zeichnet schwarzes Rechteck um BEV Bild 




		//2. Abfrage ob überhaupt Konturen erkannt wurden -> wenn nicht nutze alte Kontur bzw default werte für alte Kontur
		if (LD_contours.empty()) {
			//cout << "Keine Linien" << endl;
			LD_left_line = LD_left_line_old;
			LD_right_line = LD_right_line_old;

		}


		//3. Schleife durch alle Konturen- hier wird auf Mindestgröße geprüft und dann ein Zuordnung der der Kontur zur LeftLine oder RightLine gemacht
		for (int i = 0; i < LD_contours.size(); i++)
		{
			int area = contourArea(LD_contours[i]); //Area speichert Fläche der Aktuell betrachteten Kontur
			if ((area > LD_min_kontour_size))// && (area < LD_max_kontour_size)) // Verarbeitet nur Konturen mit ausreichender Fläche
			{
				//Die Funktion pointPolygonTest() in OpenCV überprüft die Lage eines Punktes relativ zu einem Kontur (Polygon)
				//Wenn true, gibt die Funktion den euklidischen Abstand zum nächsten Punkt des Polygons zurück
				float LD_contour_distance_left = pointPolygonTest(LD_contours[i], LD_left_line_bottom, true); // Die Funktion überprüft, wo sich der Punkt LD_left_line_bottom in Bezug auf die Kontur LD_contours[i] befindet, und gibt bei Verwendung von true zusätzlich den Abstand an
				if (LD_contour_distance_left > LD_left_line_distant_temp) // countour fits better than contour before
				{
					LD_left_line = LD_contours[i]; //Zuordnung der contour als left Line wenn näher links bottom point
					LD_left_line_distant_temp = LD_contour_distance_left;
				
				}
				float LD_contour_distance_right = pointPolygonTest(LD_contours[i], LD_right_line_bottom, true);
				if (LD_contour_distance_right > LD_right_line_distant_temp)
				{
					LD_right_line = LD_contours[i]; //Zuordnung der contour als right Line wenn näher rechter bottom point
					LD_right_line_distant_temp = LD_contour_distance_right;
					
				}
			}
		}
		//4. Abfrage wenn nur eine Linie erkannt wird -Überprüfung welche Linie fehlt und dann kopie der vorhandnen Linien um fehlende Linie zu interpolieren
		if (LD_right_line == LD_left_line) // wenn rechte = linke Line ist wurde nur einen Linie entdeckt → wir müssen die alte Linie verwenden um weiterhin zwei Linien zu haben  
		{
			Flag = true;
			//cout << "Es wurde nur eine Linie detektiert → Nutze alte Linie" << endl;
			//Bestimmt welche Linie fehlt 
			// Es wird bottom Point der einen gefunden Linie bestimmt

			single_line_bottom = LD_contourBottomPointFinder(LD_left_line);
			Point2f fallback_right_bottom = LD_contourBottomPointFinder(LD_right_line_fallback);
			Point2f fallback_left_bottom = LD_contourBottomPointFinder(LD_left_line_fallback);

			
			float distance_right = abs(norm(single_line_bottom - fallback_right_bottom));
			float distance_left = abs(norm(single_line_bottom - fallback_left_bottom));
					  
			//cout << "Distance Rechts" << distance_right <<endl;
			//cout << "Distance Links" << distance_left << endl;

					
			if (distance_right > distance_left) //Wenn Abstand der Linie zu rechts alt größer als zu links alt ist 
			{
				//cout << "Linke Linie vorhanden" << endl;
				coeffsLeft_same = LD_polyFitHelper(LD_right_line);
				coeffsRight_same = coeffsLeft_same.clone();
				coeffsRight_same.at<double>( 0 ) += 90;
			}
			else
			{
				//cout << "Rechte Linie vorhanden" << endl;
				coeffsRight_same = LD_polyFitHelper(LD_left_line);
				coeffsLeft_same = coeffsRight_same.clone();
				coeffsLeft_same.at<double>( 0 ) -= 90;
			}
		}
				
		double distance_bottom_points_x = (LD_right_line_bottom.x - LD_left_line_bottom.x);
		
		//5. Verhinderung von Sprüungen der Linien nur FallbackSprung wird zugelassen
		if ((LD_left_line_bottom.x - LD_contourBottomPointFinder(LD_left_line).x) > 50 && LD_left_line_bottom.x != 130 && distance_bottom_points_x > 40)
		{
			//cout << "Sprung LINKS verhindert" << endl;
			cout << endl;
			LD_left_line = LD_left_line_old;
		}

		if ((LD_right_line_bottom.x - LD_contourBottomPointFinder(LD_right_line).x) > 50 && LD_right_line_bottom.x != 174 && distance_bottom_points_x > 40)
		{
			//cout << "Sprung RECHTS verhindert" << endl;
			cout << endl;
			LD_right_line = LD_right_line_old;
		}
		
		
		//6. Check ob Linien zu nahe oder zu weit aneinander nicht verwerden werden Linien neu inizialisiert	   		 
		if (distance_bottom_points_x < 20 || 100 < distance_bottom_points_x) 
		{
			// reinizalizierung bottom 
			LD_left_line_bottom = { 130, 343 };
			LD_right_line_bottom = { 174, 345 };
		}
		else
		{
			LD_left_line_bottom = LD_contourBottomPointFinder(LD_left_line);
			LD_right_line_bottom = LD_contourBottomPointFinder(LD_right_line);
			
		}

		//7. Sicher Werte
		LD_left_line_old = LD_left_line;
		LD_right_line_old = LD_right_line;
		
		
		return imgLines_lokal;
	}
	//6 Funktion welche Midpoint berechnet und welche Kontur zu Points transformiert 
	Mat LD_ContourToPoint_Midpoint(Mat input_)
	{
		//Overview:
		//1. Check ob beide Linien gleich -  falls ja nutze interpolierte Linien - falls nein berechene Polycoeffs aus den vorhanden Linien  
		//2. Code zur berechnung der Krümmung des rechten Polynoms
		//3. Scheife welche alle Pixelzeilen (x von unten nach oben) 400 Zeile gesamt durchgeht ( solange x Func > 0 ist und reduziere -20pixel → alle 20 pixel Punkt) und bei x ==260 midpointy berechnet
		//4. Normierung des Midpoints auf 0 +-x → 150 weil Bild 300 Pixel breit ist (y Achse)
		//5. Sicherung der alten Werte 


		//Dekleration 
		//imshow("Midpoint_input", input_);
		Mat imgLines_lokal = input_.clone();
		double middle_point_y; //Mittelpunkt in y richtung 
		
		//1. Check ob beide Linien gleich -  falls ja nutze interpolierte Linien - falls nein berechene Polycoeffs aus den vorhanden Linien  
		if (Flag)
		{
			coeffsLeft = coeffsLeft_same;
			coeffsRight = coeffsRight_same;
			Flag = false;
		}
		else
		{
			coeffsLeft = LD_polyFitHelper(LD_left_line);
			coeffsRight = LD_polyFitHelper(LD_right_line);
		}

		//2. Code zur berechnung der Krümmung des rechten Polynoms
		double curvature_right = abs(coeffsRight.at<double>(2));
		curvature_ = curvature_right;
		//cout << " Krümmung" << curvature_right << endl;


		// Zeichnen der Konturen 
		drawContours(imgLines_lokal, vector<vector<Point>>(1, LD_right_line), -1, Scalar(0, 0, 255), FILLED); //rechte Linie BGR-Farbraum Rot
		drawContours(imgLines_lokal, vector<vector<Point>>(1, LD_left_line), -1, Scalar(255, 0, 255), FILLED);  //linke Linie BGR-Farbraum Blau

		//3. Scheife welche alle Pixelzeilen (x von unten nach oben) 400 Zeile gesamt durchgeht ( solange x Func > 0 ist und reduziere -20pixel → alle 20 pixel Punkt) und bei x ==260 midpointy berechnet
		for (int x_func = 400; x_func > 0; x_func -= 10) 
		{
			// Zeichne Punkt auf Linien von unten nach oben 
			double y_func_left = coeffsLeft.at<double>(0) + x_func * coeffsLeft.at<double>(1) + x_func * x_func *  coeffsLeft.at<double>(2); //Linie Links
			double y_func_right = coeffsRight.at<double>(0) + x_func * coeffsRight.at<double>(1) + x_func * x_func * coeffsRight.at<double>(2); //Linie Rechts
			circle(imgLines_lokal, Point(y_func_left, x_func), 3, Scalar(255, 0, 0), FILLED); //zeichne Punkte auf Linker Linie
			circle(imgLines_lokal, Point(y_func_right, x_func), 3, Scalar(255, 0, 0), FILLED);//zeichen Punkte auf Rechter Linie
			if (x_func == 260)
			{
				middle_point_y = (y_func_right + y_func_left) / 2; //Berechnung des Mittelpunktes
				// Glättung des Werts mit einem Gleitkomma-Filter
				smoothed_middle_point_y = smoothing_factor * middle_point_y + (1 - smoothing_factor) * smoothed_middle_point_y;
				circle(imgLines_lokal, Point(smoothed_middle_point_y, 260), 3, Scalar(255, 255, 0), FILLED); //Einzeichung des Mittelpunktes
			}
		}
		//4. Normierung des Midpoints auf 0 +-x → 150 weil Bild 300 Pixel breit ist (y Achse)
		LD_mid_point = smoothed_middle_point_y - 150; //Normierung 
		//5. Sicherung der alten Werte 
		LD_mid_point_old = LD_mid_point;
		return imgLines_lokal;
	}
	//7 Rücktransformation von BEV2 Normal
	Mat LD_BEV2Org(Mat inputImage_)
	{
		Mat inputPic = inputImage_.clone();
		Mat BEV_inv, imgTransformed2Normal, ReturnPic;

		// Definition der Eingangspunkte
		vector<Point2f> input_points(4);
		input_points[0] = Point2f(261, 13);   // TL (Top Left) Points2f(x,y)
		input_points[1] = Point2f(423, 14);   // TR (Top Right)
		input_points[2] = Point2f(609, 102);  // BR (Bottom Right)
		input_points[3] = Point2f(147, 99);   // BL (Bottom Left)

		// Definition der Zielpunkte
		vector<Point2f> target_points(4);
		target_points[0] = Point2f(150 - 50, 400 - 302);  // TL
		target_points[1] = Point2f(150 + 30, 400 - 277);
		target_points[2] = Point2f(150 + 24, 400 - 55);
		target_points[3] = Point2f(150 - 20, 400 - 57);

		BEV_inv = getPerspectiveTransform(target_points, input_points);  // Transformation Traget → Input also von BEV zu normal
		warpPerspective(inputPic, imgTransformed2Normal, BEV_inv, Size(720, 100));  // Ursprüngliches Format der ROI

		return imgTransformed2Normal;
	}
	//8 Mapping der Linien auf Orgbild
	Mat LD_LinesOnOrg(Mat LinePic_, Mat InputPic_)
	{
		// Erstelle eine Maske für die weißen Linien im linken Teil des Bildes
		Mat mask;
		Mat gray;
		copyMakeBorder(LinePic_, LinePic_, 260, 512 - 360, 0, 0, 0, Scalar(0, 0, 0));  // Das Bild wird vertikal um 260 Pixel oben und 152
		threshold(LinePic_, mask, 100, 255, THRESH_BINARY);  // Schwellenwert anpassen, um die Linien zu isolieren
		//imshow("Lines_lokal", LinePic_);
		
		// Konvertiere InputPic_ falls nötig in ein 3-Kanal-Bild
		if (InputPic_.channels() == 1)
		{
			cvtColor(InputPic_, InputPic_, COLOR_GRAY2BGR);
		}
		else if (InputPic_.channels() == 4)
		{
			cvtColor(InputPic_, InputPic_, COLOR_BGRA2BGR);
		}
		// Stelle sicher, dass beide Bilder die gleiche Größe haben
		if (InputPic_.size() != LinePic_.size())
		{
			resize(LinePic_, LinePic_, InputPic_.size());
		}
		// Konvertiere beide Bilder in denselben Typ
		InputPic_.convertTo(InputPic_, CV_8UC3);
		LinePic_.convertTo(LinePic_, CV_8UC3);
		// Überlagere beide Bilder
		Mat result;
		addWeighted(InputPic_, 0.8, LinePic_, 1.0, 0, result);  // Verändere die Gewichte (z. B. 2.0 oder 0.5) für stärkere/schwächere Farben
		return result;
	}
	//Helper Functions
	//PolyFit Helper Function 
	Mat LD_polyFitHelper(vector<Point> points)
	{
		// closed form solution with possibility to change order
		// beta = parameters
		// X = features x samples
		// Y = label
		Mat X(points.size(), (LD_poly_order + 1), CV_64F);
		Mat Y(points.size(), 1, CV_64F);
		for (int i = 0; i < X.rows; i++)
		{
			for (int j = 0; j < X.cols; j++)
			{
				X.at<double>(i, j) = pow(points[i].y, j);
			}
		}
		for (int i = 0; i < Y.rows; i++)
		{
			Y.at<double>(i, 0) = points[i].x;
		}
		Mat K((LD_poly_order + 1), 1, CV_64F);
		if (X.data != NULL)
		{
			K = (X.t() * X).inv() * X.t() * Y;
		}
		return K;
	}
	//Helper Function um den untersten Punkt einer Contour zu finden  = Startpunkt des polynominal 
	Point2f LD_contourBottomPointFinder(vector<Point> contour)
	{
		double most_bottom_point = 0;
		int most_bottom_index;
		for (int i = 0; i < contour.size(); i++)
		{
			if (contour[i].y > most_bottom_point)
			{
				most_bottom_point = contour[i].y;
				most_bottom_index = i;
			}
			//letzter Punkt bei dem gilt countour.y > 0 ist der unterste Point 
			//von diesem wird dann der x und y value returned
		}
		Point2f countour_bottom_point = { (float)contour[most_bottom_index].x, (float)contour[most_bottom_index].y };
		return countour_bottom_point;
	}
	// Function um openCV Image anzuzeigen
	void LD_showImage(string name_, Mat img_)
	{
		namedWindow(name_, WINDOW_AUTOSIZE);
		imshow(name_, img_);
		waitKey(1);
	}
	double LD_getCurvature()
	{
		return curvature_;
	}
	

};

// Klasse welche Quer und Längsregelung übernimmmt → FERTIG
class LaneAssistant
{
	// insert your custom functions and algorithms here
public:
	LaneAssistant()
	{
	}
	//Senden der Werte an Tronis erst steeringstring ; Throttelstring
	bool processData(tronis::CircularMultiQueuedSocket& socket)
	{
		string command = to_string(steeringControl.SC_TronisCommand()) + ";" + to_string(throttleControl.TC_TronisCommand());
		socket.send(tronis::SocketData(command));
		return true;
	}
protected:
	std::string image_name_;
	cv::Mat image_;
	tronis::LocationSub ego_location_;
	tronis::OrientationSub ego_orientation_;
	double ego_velocity_;
	


	//double previous_time = 0.0; // Speichert die Zeit des vorherigen Frames
	// Function to detect lanes based on camera image
	// Insert your algorithm here
	
	
	
	LaneDetection laneDetection;
	SteeringControl steeringControl;
	ThrottleControl throttleControl = ThrottleControl(true);

	//Ausführung aller Classen
	void detectLanes()
	{
		//Hier wird OG_Image übertragen von processImage 
		//und die Kantenerkennung mit Mittelsollwert berechnet
		double LA_midpoint_delta = laneDetection.LaneFinderPipeline(image_);
		double curvature_right = laneDetection.LD_getCurvature();
		//Anschließend wird der Steering Sollwert berechnet
		steeringControl.SC_Control(LA_midpoint_delta, ego_velocity_);
		//Final der Throttle Sollwert
		throttleControl.TC_control(ego_velocity_, curvature_right);
	}
	bool processPoseVelocity(tronis::PoseVelocitySub* msg)
	{
		ego_location_ = msg->Location;
		ego_orientation_ = msg->Orientation;
		ego_velocity_ = msg->Velocity;
		return true;
	}
	
	//Gibt Struktur der Map vor 
	struct ObjectInfo 
	{
		double distance;
		double angle;
		double x, y;  // Position für Tracking
		int klasse_;
		chrono::steady_clock::time_point timestamp;
	};

	//Hilfsfunktion um nächstes vorheriges Objekt zu finden → wichtig falls neue Objekte erkannt werden wird nicht nur nach Schleifen id gefiltert
	int findClosestPreviousObject(double x, double y, const unordered_map<int, ObjectInfo>& previousDistances) {
		int bestMatch = -1;
		double minDistance = numeric_limits<double>::max();

		for (const auto& entry : previousDistances)
		{
			int prevID = entry.first;                  // ID des vorherigen Objekts
			const ObjectInfo& prevData = entry.second; // Daten des vorherigen Objekts

			double dist = sqrt(std::pow(prevData.x - x, 2) + pow(prevData.y - y, 2));
			if (dist < minDistance) {
				minDistance = dist;
				bestMatch = prevID;
			}
		}
		return bestMatch;
	}

	//Erstellung der Map für vorherige Werte
	unordered_map<int, ObjectInfo> previousDistances;


	//Objekterkennungsfunktion 
	bool processObject(tronis::ModelDataWrapper data_model) {
		//liest werte von Sensoren ein
		tronis::BoxDataSub* sensorData = data_model.get_typed<tronis::BoxDataSub>();

		double LA_vehicleDistance;
		
		double lowest_distance = 1000; 
		double ego_velo = ego_velocity_ * 0.036;
		double relativeSpeed = 0;
		double relativeSpeed_axial = 0;
		double objectAngle = 0;   
		double objectAngle_rad = 0;
		double Delta_Speed = 0; //Delta zwischen relativ Geschwindigkeit und Ego_Velo
		double foundDistance = -1; // Initialwert für "nicht gefunden"
		double minDistance = std::numeric_limits<double>::max(); // Initialwert für die kleinste Distanz
		
		int klasse = 0;		//int für Objektklassen ID

		bool found = false;	//Bool welches sagt ob Vordermann erkannt wurde 
		bool LA_vehicleDetected = false; //Dekleration des Returnwerts

		//prints		
		//cout << "\033[2J\033[H"; // Clear Terminal 
		//cout << " Anzahl der Objekte: " << sensorData->Objects.size() << endl;
		
		//Erstellung der Map für jetzige Werte
		unordered_map<int, ObjectInfo> currentDistances;
		

		//For Schleife über alle Objekte 
		for (int i = 0; i < sensorData->Objects.size(); i++)
		{
			//Objektvariabeln werden erstellt 
			tronis::ObjectSub& object = sensorData->Objects[i];
			string actorName = object.ActorName.Value();
			
			
			
			

			// Bounding-Box-Prüfung 
			if ((object.BB.Extends.X > 30) && (object.BB.Extends.X < 800) && (object.BB.Extends.Y > 30) && (object.BB.Extends.Y < 800) && (object.ActorName != "SnappyRoad") && (object.ActorName != "SkySphere") && (object.ActorName != "TronisTrack"))  // Filterung der Objekte nach Entferenung in x und Y
			{
				//Abstand in x und y von Objekt zu Egofzg in Meter
				double objectX = object.Pose.Location.X / 100.0;
				double objectY = object.Pose.Location.Y / 100.0;


				//Abstand via pythagoras
				LA_vehicleDistance = sqrt(pow(objectX, 2) + pow(objectY, 2));

				//objektwinkel in Deg
				objectAngle = atan2(objectY, objectX) * (180.0 / 3.14159265358979323846);
				//objektwinkel in Rad
				objectAngle_rad = atan2(objectY, objectX);

				//cout << " Schleife i:  " << i << " Name: " << actorName << endl;


				//Wenn Abstand größer als 1 ist 
				if (LA_vehicleDistance > 1)
				{
					//Speicherung der aktuellen Zeit
					auto currentTime = std::chrono::steady_clock::now();
					
					//suchen des nächsten Objektes in der map previousDistances und verwendung dieses
					int closestObject = findClosestPreviousObject(objectX, objectY, previousDistances);
					
					//abfrage ob nächstes Objekt existiert
					if (closestObject != -1)
					{
						//vorherigen Werte des nächsten Elements verwenden
						auto prevData = previousDistances[closestObject];

						//Zeitdifferenz berechnen 
						double timeDiff = std::chrono::duration<double>(currentTime - prevData.timestamp).count();
						



						if (timeDiff > 0 && timeDiff < 2.0) 
						{ // Zeitdifferenz-Prüfung für Stabilität
							
							//Relativgeschwindigkeiten in x und y Richtung berechen 
							double v_rel_x = ((objectX - prevData.x) / timeDiff) * 3.6;
							double v_rel_y = ((objectY - prevData.y) / timeDiff) * 3.6;

							
							//Radiale Relativgeschwindigkeit berechen 
							relativeSpeed = v_rel_x * cos(objectAngle_rad) + v_rel_y * sin(objectAngle_rad);
							//Axiale Relativgeschwindigkeit berechen 
							relativeSpeed_axial = ((LA_vehicleDistance - prevData.distance) / timeDiff) * 3.6;
							//Delta Speed berechenen 
							Delta_Speed = abs(relativeSpeed) - abs(ego_velo);


							//Printausgaben 
							/*cout << "Alte Distanz: " << prevData.distance << " Neue Distanz: " << LA_vehicleDistance << " Delta Zeit: "<< timeDiff<< endl;
							cout << "Objekt: " << actorName <<  " Relativgeschwindigkeit radial: " << relativeSpeed << " km/h und Winkel von " << objectAngle << " Grad" << endl;
							cout << "Objekt: " << actorName << " Relativgeschwindigkeit axial: " << relativeSpeed_axial << " km/h und Winkel von " << objectAngle << " Grad" << endl;
							cout << "Eigenspeed " << ego_velo << " DELTA SPEED: " << Delta_Speed << endl;*/
							
							
						}
						
					}
					
					
					// Gegenverkehr erkennen besser
					//if ((relativeSpeed_axial <= 0) && (abs(relativeSpeed_axial) > ego_velo))
					// Gegenverkehr erkennen und stehden Vordermann
					if ((relativeSpeed_axial <= 0) && (abs(relativeSpeed_axial) > ego_velo) && (abs(objectAngle) > 5))
					{
						//Gegenverkehr CASE
										
						//cout << actorName << " ist Gegenverkehr : 1" << endl;
						klasse = 1;
						LA_vehicleDetected = false;
						//lowest Distance check 
						if (LA_vehicleDistance < lowest_distance)
						{
							lowest_distance = LA_vehicleDistance;
							currentDistances[i] = { lowest_distance, objectAngle, objectX, objectY, klasse, currentTime };
						}
						else
						{
							currentDistances[i] = { LA_vehicleDistance, objectAngle, objectX, objectY, klasse, currentTime };
						}
						continue;
							
							
						
						
					}
					// Stehende Autos
					if ((abs(Delta_Speed) < 5) && (abs(objectAngle) > 10))
					{
						//cout << actorName << " ist Stehendes Objekt: 2" << endl;
						klasse = 2;
						LA_vehicleDetected = false;
						//lowest Distance check 
						if (LA_vehicleDistance < lowest_distance)
						{
							lowest_distance = LA_vehicleDistance;
							currentDistances[i] = { lowest_distance, objectAngle, objectX, objectY, klasse, currentTime };
						}
						else
						{
							currentDistances[i] = { LA_vehicleDistance, objectAngle, objectX, objectY, klasse, currentTime };
						}
						continue;
						
						
						
					}
					// Vordermann-Erkennung
					else
					{
															
						//cout << actorName << " ist Vordermann: 3" << endl;
						klasse = 3;
						LA_vehicleDetected = true;
						if (LA_vehicleDistance < lowest_distance)
						{
							lowest_distance = LA_vehicleDistance;
							currentDistances[i] = { lowest_distance, objectAngle, objectX, objectY, klasse, currentTime };
						}
						else
						{
							currentDistances[i] = { LA_vehicleDistance, objectAngle, objectX, objectY, klasse, currentTime };
						}
					}					
				}
			}
		}
		// Iteriere über alle Einträge in currentDistances um Vordermann Key zu finden 
		for (const auto& entry : currentDistances)
		{
			const int key = entry.first;           // Key (z. B. Objekt-ID)
			const ObjectInfo& value = entry.second; // Wert (Objektdaten)

			// Prüfe, ob das Objekt die Klasse 3 hat und ob es näher ist als bisherige Distanzen
			if (value.klasse_ == 3 && value.distance < minDistance)
			{
				minDistance = value.distance;  // Aktualisiere die kleinste Distanz
				foundDistance = minDistance;  // Speichere die gefundene Distanz
				
				//print
				//cout << foundDistance-6 << " verwendete Distance" << endl;
				
				found = true;                 // Markiere, dass ein Treffer gefunden wurde

				throttleControl.TC_set_distance(foundDistance - 6); //Boundingbox beachtet
				throttleControl.TC_set_detected(found);
			}
		}
		//aktuelle Distance wird vorherige Distance
		previousDistances = move(currentDistances);
		
		


		//Case falls kein Vordermann gefunden wurde 
		if (!found)
		{
			//print 
			//cout << "nichts relevantes" << endl;
			throttleControl.TC_set_distance(9999);
		}
		//cout << "#######################" << endl;
		return LA_vehicleDetected;
	}

public:
	// Function to process received tronis data
	bool getData(tronis::ModelDataWrapper data_model)
	{
		if (data_model->GetModelType() == tronis::ModelType::Tronis)
		{
			//Ausgabe alle Objekte
			//std::cout << "Id: " << data_model->GetTypeId() << ", Name: " << data_model->GetName()<< ", Time: " << data_model->GetTime() << std::endl;
			// if data is sensor output, process data
			switch (static_cast<tronis::TronisDataType>(data_model->GetDataTypeId()))
			{
			case tronis::TronisDataType::Image:
			{
				processImage(data_model->GetName(), data_model.get_typed<tronis::ImageSub>()->Image);
				break;
			}
			case tronis::TronisDataType::ImageFrame:
			{
				const tronis::ImageFrame& frames(data_model.get_typed<tronis::ImageFrameSub>()->Images);
				for (size_t i = 0; i != frames.numImages(); ++i)
				{
					std::ostringstream os;
					os << data_model->GetName() << "_" << i + 1;
					processImage(os.str(), frames.image(i));
				}
				break;
			}
			case tronis::TronisDataType::ImageFramePose:
			{
				const tronis::ImageFrame& frames(data_model.get_typed<tronis::ImageFramePoseSub>()->Images);
				for (size_t i = 0; i != frames.numImages(); ++i)
				{
					std::ostringstream os;
					os << data_model->GetName() << "_" << i + 1;
					processImage(os.str(), frames.image(i));
				}
				break;
			}
			case tronis::TronisDataType::PoseVelocity:
			{
				processPoseVelocity(data_model.get_typed<tronis::PoseVelocitySub>());
				break;
			}
			case tronis::TronisDataType::Object:
			{
				processObject(data_model);
				break;
			}
			default:
			{
				
				//WICHTIGE ÄNDERUNG !!!!!!
				//std::cout << data_model->ToString() << std::endl;
				
				processObject(data_model);
				break;
			}
			}
			return true;
		}
		else
		{
			std::cout << data_model->ToString() << std::endl;
			return false;
		}
	}
protected:
	// Function to show an openCV image in a separate window
	void showImage(std::string image_name, cv::Mat image)
	{
		cv::Mat out = image;
		if (image.type() == CV_32F || image.type() == CV_64F)
		{
			cv::normalize(image, out, 0.0, 1.0, cv::NORM_MINMAX, image.type());
		}
		/////////////////////////////////////////////////////////////////////////////////
		//!!!Hier Auskommentieren um default Bildanzeige zu übergehen!!!
		// cv::namedWindow( image_name.c_str(), cv::WINDOW_NORMAL );
		// cv::imshow( image_name.c_str(), out );
		////////////////////////////////////////////////////////////////////////////////
	}
	// Function to convert tronis image to openCV image
	bool processImage(const std::string& base_name, const tronis::Image& image)
	{
		//std::cout << "processImage" << std::endl;
		if (image.empty())
		{
			std::cout << "empty image" << std::endl;
			return false;
		}
		image_name_ = base_name;
		image_ = tronis::image2Mat(image);
		detectLanes();
		showImage(image_name_, image_);
		return true;
	}
};

// main loop opens socket and listens for incoming data
int main(int argc, char** argv)
{
	std::cout << "Welcome to lane assistant" << std::endl;
	// specify socket parameters
	std::string socket_type = "TcpSocket";
	std::string socket_ip = "127.0.0.1";
	std::string socket_port = "7778";
	std::ostringstream socket_params;
	socket_params << "{Socket:\"" << socket_type << "\", IpBind:\"" << socket_ip << "\", PortBind:" << socket_port << "}";
	int key_press = 0;  // close app on key press 'q'
	tronis::CircularMultiQueuedSocket msg_grabber;
	uint32_t timeout_ms = 500;  // close grabber, if last received msg is older than this param
	LaneAssistant lane_assistant;
	while (key_press != 'q')
	{
		std::cout << "Wait for connection..." << std::endl;
		msg_grabber.open_str(socket_params.str());
		if (!msg_grabber.isOpen())
		{
			printf("Failed to open grabber, retry...!\n");
			continue;
		}
		std::cout << "Start grabbing" << std::endl;
		tronis::SocketData received_data;
		uint32_t time_ms = 0;
		while (key_press != 'q')
		{
			// wait for data, close after timeout_ms without new data
			if (msg_grabber.tryPop(received_data, true))
			{
				// data received! reset timer
				time_ms = 0;
				// convert socket data to tronis model data
				tronis::SocketDataStream data_stream(received_data);
				tronis::ModelDataWrapper data_model(tronis::Models::Create(data_stream, tronis::MessageFormat::raw));
				if (!data_model.is_valid())
				{
					std::cout << "received invalid data, continue..." << std::endl;
					continue;
				}
				// identify data type
				lane_assistant.getData(data_model);
				lane_assistant.processData(msg_grabber);
			}
			else
			{
				// no data received, update timer
				++time_ms;
				if (time_ms > timeout_ms)
				{
					std::cout << "Timeout, no data" << std::endl;
					msg_grabber.close();
					break;
				}
				else
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
					key_press = cv::waitKey(1);
				}
			}
		}
		msg_grabber.close();
	}
	return 0;
}
