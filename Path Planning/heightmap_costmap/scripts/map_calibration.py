#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.offsetbox import AnchoredText
import matplotlib.patheffects as path_effects
import ast
import csv 

class MapCalibration:
    def __init__(self):
        rospy.init_node('map_calibration_node', anonymous=True)
        image_path = rospy.get_param('~image_path')
        real_coords = ast.literal_eval(rospy.get_param('~real_coords'))
        self.input_csv = rospy.get_param('~input_csv', None)
        self.output_csv = rospy.get_param('~output_csv', None)
        
        # Check if CSV paths are provided and not empty
        if self.input_csv == "":
            self.input_csv = None
        if self.output_csv == "":
            self.output_csv = None
        
        # Load image and detect orange spots
        self.image = cv2.imread(image_path)
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([5, 100, 100])
        upper_orange = np.array([15, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:3]
        centroids = []
        for cnt in contours:
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                centroids.append((cx, cy))
        centroids.sort(key=lambda x: (x[1], x[0]))
        self.pixel_coords = centroids
        self.real_coords = real_coords
        
        # Origin and scale calculation
        self.origin_idx = np.argmax([c[1] for c in centroids])
        self.origin_px = centroids[self.origin_idx]
        self.origin_real = real_coords[self.origin_idx]
        scales_x, scales_y = [], []
        for i, (px, real) in enumerate(zip(centroids, real_coords)):
            if i == self.origin_idx:
                continue
            rx = real[0] - self.origin_real[0]
            ry = real[1] - self.origin_real[1]
            delta_px = px[0] - self.origin_px[0]
            delta_py = self.origin_px[1] - px[1]
            if rx != 0:
                scales_x.append(delta_py / ry)
            if ry != 0:
                scales_y.append(delta_py / ry)
        self.scale_x = np.mean(scales_y)
        self.scale_y = np.mean(scales_y)
        
        # Visualization setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.ax.imshow(self.gray_image, cmap='gray')
        self.ax.grid(True, color='blue', linestyle='--', linewidth=0.5)
        self.ax.set_title("Calibration Visualization")
        self.ax.plot(self.origin_px[0], self.origin_px[1], 'ro', markersize=10)
        self.ax.text(self.origin_px[0], self.origin_px[1], '(0,0)', color='red')
        for i, (px, real) in enumerate(zip(centroids, real_coords)):
            if i == self.origin_idx:
                continue
            adjusted_real = (real[0]-self.origin_real[0], real[1]-self.origin_real[1])
            self.ax.plot(px[0], px[1], 'yo', markersize=10)
            self.ax.text(px[0], px[1], f'({adjusted_real[0]}, {adjusted_real[1]})', color='yellow')
        plt.draw()
        
        #Add help text
        help_text = "Enter:\n- 'r x,y' for real→pixel\n- 'p x,y' for pixel→real"
        self.ax.add_artist(AnchoredText(help_text, loc='upper right'))
        plt.draw()

    def real_to_pixel(self, rx, ry):
        """Convert real-world coordinates to pixel coordinates"""
        px = self.origin_px[0] + self.scale_x * rx
        py = self.origin_px[1] - self.scale_y * ry
        return px, py

    def pixel_to_real(self, px, py):
        """Convert pixel coordinates to real-world coordinates"""
        rx = (px - self.origin_px[0]) / self.scale_x
        ry = (self.origin_px[1] - py) / self.scale_y
        return rx, ry

    # Add the CSV processing method
    def process_csv(self):
        if not self.input_csv or not self.output_csv:
            rospy.logwarn("Input or output CSV path not provided, skipping CSV processing.")
            return
        try:
            with open(self.input_csv, 'r') as infile, open(self.output_csv, 'w') as outfile:
                reader = csv.DictReader(infile)
                if 'x_pixel' not in reader.fieldnames or 'y_pixel' not in reader.fieldnames:
                    rospy.logerr("CSV must contain 'x_pixel' and 'y_pixel' columns")
                    return
                # Prepare output fields
                fieldnames = reader.fieldnames + ['real_x', 'real_y']
                writer = csv.DictWriter(outfile, fieldnames=fieldnames)
                writer.writeheader()
                for row in reader:
                    try:
                        x_pixel = int(row['x_pixel'])
                        y_pixel = int(row['y_pixel'])
                    except ValueError as e:
                        rospy.logwarn(f"Skipping row {row.get('index', 'unknown')}: {e}")
                        continue
                    # Convert to real coordinates
                    real_x, real_y = self.pixel_to_real(x_pixel, y_pixel)
                    row['real_x'] = f"{real_x:.4f}"
                    row['real_y'] = f"{real_y:.4f}"
                    writer.writerow(row)
                rospy.loginfo(f"CSV processing complete. Output saved to {self.output_csv}")
        except Exception as e:
            rospy.logerr(f"Error processing CSV: {str(e)}")


    def run(self):
        
                # Process CSV on startup if paths are provided
        if self.input_csv and self.output_csv:
            self.process_csv()
        
        print("Ready for coordinate conversions (multiple commands separated by ';')...")
        while not rospy.is_shutdown():
            try:
                user_input = input("Enter command(s): ").strip()
                if not user_input:
                    continue
                
                # Split into multiple commands
                cmd_parts = user_input.split(';')
                
                for cmd_part in cmd_parts:
                    cmd_part = cmd_part.strip()
                    if not cmd_part:
                        continue
                    
                    try:
                        parts = cmd_part.split(maxsplit=1)
                        if len(parts) < 2:
                            raise ValueError("Invalid command format")
                        
                        cmd = parts[0].lower()
                        coords = list(map(float, parts[1].split(',')))
                        
                        if cmd == 'r' and len(coords) == 2:  # Real to pixel
                            rx, ry = coords
                            px, py = self.real_to_pixel(rx, ry)
                            print(f"[R→P] ({rx},{ry}) → ({px:.2f}, {py:.2f})")
                            self.ax.plot(px, py, 'go', markersize=10)
                            text_obj = self.ax.text(px, py, f'R({rx},{ry})', color='green', fontsize=6)
                            text_obj.set_path_effects([path_effects.Stroke(linewidth=1, foreground='black'),path_effects.Normal()])
                        
                        elif cmd == 'p' and len(coords) == 2:  # Pixel to real
                            px, py = coords
                            rx, ry = self.pixel_to_real(px, py)
                            print(f"[P→R] ({px},{py}) → ({rx:.2f}, {ry:.2f})")
                            self.ax.plot(px, py, 'ms', markersize=10)
                            self.ax.text(px, py, f'P({rx:.1f},{ry:.1f})', color='magenta')
                            
                        
                        else:
                            raise ValueError("Invalid command or coordinates")
                        
                    except Exception as e:
                        print(f"Error processing '{cmd_part}': {str(e)}")
                
                plt.draw()
            
            except Exception as e:
                print(f"General error: {str(e)}")
        
        plt.show(block=True)

if __name__ == '__main__':
    try:
        calib = MapCalibration()
        calib.run()
    except rospy.ROSInterruptException:
        pass