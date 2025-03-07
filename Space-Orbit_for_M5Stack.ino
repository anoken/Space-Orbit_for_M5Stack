#include <M5Unified.h>
#include <vector>
#include <cmath>

// Constants
const int EARTH_RADIUS = 15;  // Earth radius (pixels)
const int MOON_RADIUS = 5;    // Moon radius (pixels)
const int MOON_DISTANCE = 100; // Moon orbit radius (pixels)
const double MOON_ANGULAR_VELOCITY = 0.005; // Moon angular velocity
const int TRAIL_LENGTH = 50;  // Orbit trail length (points)

// Gravity constants
const double MU_EARTH = 2500;  // Earth gravity constant
const double MU_MOON = 250;    // Moon gravity constant

// Colors (RGB565 format for M5Stack)
const uint16_t EARTH_COLOR = M5.Lcd.color565(255, 100, 0);  // Earth color (orange)
const uint16_t MOON_COLOR = M5.Lcd.color565(255, 255, 0);   // Moon color (yellow)
const uint16_t BOUND_ORBIT_COLOR = M5.Lcd.color565(0, 255, 0);  // Bound orbit color (green)
const uint16_t ESCAPE_ORBIT_COLOR = M5.Lcd.color565(0, 255, 255); // Escape orbit color (cyan)
const uint16_t SPACECRAFT_COLOR = M5.Lcd.color565(255, 255, 255); // Spacecraft color (white)
const uint16_t TRAIL_COLOR = M5.Lcd.color565(100, 100, 100); // Trail color (gray)
const uint16_t BG_COLOR = TFT_BLACK;  // Background color

// Screen size (M5Stack Core S3 LCD)
const int WIDTH = 320;
const int HEIGHT = 240;

// Global variables
double moonAngle = 0;  // Moon angle
double moonX = 0, moonY = 0;  // Moon position

// Create sprite for double buffering
LGFX_Sprite* sprite = nullptr;

// Vector class (simplified p5.js Vector implementation)
class Vector {
public:
    double x, y;

    Vector(double x = 0, double y = 0) : x(x), y(y) {}

    Vector& addTo(const Vector& vec) {
        x += vec.x;
        y += vec.y;
        return *this;
    }

    Vector& subFrom(const Vector& vec) {
        x -= vec.x;
        y -= vec.y;
        return *this;
    }

    Vector& multBy(double scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    double mag() const {
        return std::sqrt(x * x + y * y);
    }

    Vector& normalize() {
        double m = mag();
        if (m != 0) {
            x /= m;
            y /= m;
        }
        return *this;
    }

    static Vector add(const Vector& vec1, const Vector& vec2) {
        return Vector(vec1.x + vec2.x, vec1.y + vec2.y);
    }

    static Vector sub(const Vector& vec1, const Vector& vec2) {
        return Vector(vec1.x - vec2.x, vec1.y - vec2.y);
    }

    static Vector mult(const Vector& vec, double scalar) {
        return Vector(vec.x * scalar, vec.y * scalar);
    }
};

// Forward declarations
class Spacecraft;

// Global containers
std::vector<Vector> moonTrail;  // Moon orbit history
std::vector<Spacecraft> spacecrafts;  // Spacecraft array

// Sparkle effect structure
struct SparkleEffect {
    double x, y;
    int life;
};

// Explosion effect structure
struct ExplosionEffect {
    double x, y;
    int size;
    int life;
    double alpha;
};

std::vector<SparkleEffect> sparkleEffects;
std::vector<ExplosionEffect> explosionEffects;

// Spacecraft class
class Spacecraft {
public:
    Vector position;
    Vector velocity;
    std::vector<Vector> trail;
    double energy;
    double prevEnergy;
    bool sparkle;

    Spacecraft(double x, double y, double vx, double vy)
        : position(x, y), velocity(vx, vy), energy(0), prevEnergy(0), sparkle(false) {}

    // Update position and velocity (Runge-Kutta method)
    bool update() {
        // Check collision with Earth
        double distToEarth = std::sqrt(std::pow(position.x - WIDTH / 2, 2) + std::pow(position.y - HEIGHT / 2, 2));
        if (distToEarth <= EARTH_RADIUS) {
            // Add explosion effect
            ExplosionEffect explosion;
            explosion.x = position.x;
            explosion.y = position.y;
            explosion.size = 15;
            explosion.life = 30;
            explosion.alpha = 255;
            explosionEffects.push_back(explosion);
            
            // Return true to indicate this spacecraft should be removed
            return true;
        }
        
        // Check collision with Moon
        double distToMoon = std::sqrt(std::pow(position.x - moonX, 2) + std::pow(position.y - moonY, 2));
        if (distToMoon <= MOON_RADIUS) {
            // Add explosion effect
            ExplosionEffect explosion;
            explosion.x = position.x;
            explosion.y = position.y;
            explosion.size = 10;  // Smaller explosion on moon
            explosion.life = 30;
            explosion.alpha = 255;
            explosionEffects.push_back(explosion);
            
            // Return true to indicate this spacecraft should be removed
            return true;
        }
        
        // Save current Earth-centered energy
        prevEnergy = energy;

        // Runge-Kutta method (RK4) for numerical integration
        Vector k1v = calculateAcceleration(position);
        Vector k1r = Vector::mult(velocity, 1);

        Vector tempPos = Vector::add(position, Vector::mult(k1r, 0.5));
        Vector tempVel = Vector::add(velocity, Vector::mult(k1v, 0.5));
        Vector k2v = calculateAcceleration(tempPos);
        Vector k2r = Vector::mult(tempVel, 1);

        tempPos = Vector::add(position, Vector::mult(k2r, 0.5));
        tempVel = Vector::add(velocity, Vector::mult(k2v, 0.5));
        Vector k3v = calculateAcceleration(tempPos);
        Vector k3r = Vector::mult(tempVel, 1);

        tempPos = Vector::add(position, k3r);
        tempVel = Vector::add(velocity, k3v);
        Vector k4v = calculateAcceleration(tempPos);
        Vector k4r = Vector::mult(tempVel, 1);

        // Update velocity and position
        Vector dv = Vector::mult(
            Vector::add(
                Vector::add(k1v, Vector::mult(k2v, 2)),
                Vector::add(Vector::mult(k3v, 2), k4v)
            ),
            1.0/6.0
        );
        Vector dr = Vector::mult(
            Vector::add(
                Vector::add(k1r, Vector::mult(k2r, 2)),
                Vector::add(Vector::mult(k3r, 2), k4r)
            ),
            1.0/6.0
        );

        velocity.addTo(dv);
        position.addTo(dr);

        // Update orbit trail
        trail.push_back(Vector(position.x, position.y));
        if (trail.size() > TRAIL_LENGTH) {
            trail.erase(trail.begin());
        }

        // Calculate Earth-centered energy
        double rE = std::sqrt(std::pow(position.x - WIDTH / 2, 2) + std::pow(position.y - HEIGHT / 2, 2));
        double v = velocity.mag();
        energy = 0.5 * v * v - MU_EARTH / rE;

        // Check for sparkle effect (if energy increase exceeds threshold)
        double energyIncrease = energy - prevEnergy;
        if (energyIncrease > 0.1) {
            sparkle = true;
            // Add sparkle effect
            SparkleEffect effect;
            effect.x = position.x;
            effect.y = position.y;
            effect.life = 20;  // Effect lifetime (frames)
            sparkleEffects.push_back(effect);
        } else {
            sparkle = false;
        }
        
        return false;
    }

    // Calculate acceleration (from Earth and Moon gravity)
    Vector calculateAcceleration(const Vector& pos) {
        // Gravity from Earth
        Vector rEarth(WIDTH / 2 - pos.x, HEIGHT / 2 - pos.y);
        double rEarthMag = rEarth.mag();
        Vector aEarth = Vector::mult(rEarth, MU_EARTH / (rEarthMag * rEarthMag * rEarthMag));

        // Gravity from Moon
        Vector rMoon(moonX - pos.x, moonY - pos.y);
        double rMoonMag = rMoon.mag();
        Vector aMoon = Vector::mult(rMoon, MU_MOON / (rMoonMag * rMoonMag * rMoonMag));

        // Total acceleration
        return Vector::add(aEarth, aMoon);
    }

    // Draw spacecraft and its orbit to sprite
    void draw(LGFX_Sprite* spr) {
        // Determine orbit color based on energy
        uint16_t orbitColor = (energy < 0) ? BOUND_ORBIT_COLOR : ESCAPE_ORBIT_COLOR;

        // Draw orbit trail
        if (trail.size() > 1) {
            for (size_t i = 1; i < trail.size(); i++) {
                spr->drawLine(
                    static_cast<int>(trail[i-1].x), static_cast<int>(trail[i-1].y),
                    static_cast<int>(trail[i].x), static_cast<int>(trail[i].y),
                    orbitColor);
            }
        }

        // Draw spacecraft
        spr->fillCircle(static_cast<int>(position.x), static_cast<int>(position.y), 2, SPACECRAFT_COLOR);
    }
};

// Update moon position
void updateMoonPosition() {
    moonX = WIDTH / 2 + MOON_DISTANCE * std::cos(moonAngle);
    moonY = HEIGHT / 2 + MOON_DISTANCE * std::sin(moonAngle);
    
    // Update moon orbit history
    moonTrail.push_back(Vector(moonX, moonY));
    if (moonTrail.size() > 360) {  // Keep about 1 orbit (360 points)
        moonTrail.erase(moonTrail.begin());
    }
}

// Add spacecraft at touch position
void addSpacecraft(int x, int y) {
    // If Earth is clicked, clear all spacecraft
    double dEarth = std::sqrt(std::pow(x - WIDTH / 2, 2) + std::pow(y - HEIGHT / 2, 2));
    if (dEarth < EARTH_RADIUS) {
        spacecrafts.clear();
        sparkleEffects.clear();
        return;
    }
    
    // Calculate initial velocity from touch position (tangential to center)
    double dirX = x - WIDTH / 2;
    double dirY = y - HEIGHT / 2;
    double dirMag = std::sqrt(dirX * dirX + dirY * dirY);
    
    // Velocity magnitude proportional to distance (farther = faster)
    double speed = 1 + 3 * (dirMag / (WIDTH / 2));
    
    // Normalize direction vector and set velocity (90 degree rotation for tangential motion)
    double vx = (dirY) / dirMag * speed;
    double vy = (-dirX) / dirMag * speed;
    
    // Add spacecraft
    spacecrafts.push_back(Spacecraft(x, y, vx, vy));
}

// Draw everything to sprite
void drawToSprite() {
    if (sprite == nullptr) return;
    
    // Clear sprite
    sprite->fillScreen(BG_COLOR);
    
    // Draw moon orbit
    if (moonTrail.size() > 1) {
        for (size_t i = 1; i < moonTrail.size(); i++) {
            sprite->drawLine(
                static_cast<int>(moonTrail[i-1].x), static_cast<int>(moonTrail[i-1].y),
                static_cast<int>(moonTrail[i].x), static_cast<int>(moonTrail[i].y),
                TRAIL_COLOR);
        }
    }
    
    // Draw Earth
    sprite->fillCircle(WIDTH / 2, HEIGHT / 2, EARTH_RADIUS, EARTH_COLOR);
    
    // Draw moon
    sprite->fillCircle(static_cast<int>(moonX), static_cast<int>(moonY), MOON_RADIUS, MOON_COLOR);
    
    // Draw spacecraft
    for (auto& spacecraft : spacecrafts) {
        spacecraft.draw(sprite);
    }
    
    // Draw explosion effects
    for (auto& effect : explosionEffects) {
        // Draw effect - simplified for M5Stack
        int size = effect.size * (1 - effect.life / 30.0);
        
        // Draw explosion using concentric circles with fading alpha
        uint16_t explosion_color = M5.Lcd.color565(255, 
                                                 static_cast<uint8_t>(255 * (1 - effect.life / 30.0)), 
                                                 0);
        
        sprite->fillCircle(static_cast<int>(effect.x), 
                         static_cast<int>(effect.y), 
                         size, 
                         explosion_color);
    }
    
    // Draw sparkle effects
    for (auto& effect : sparkleEffects) {
        // Draw effect - simple star shape (cross)
        int x = static_cast<int>(effect.x);
        int y = static_cast<int>(effect.y);
        int size = 3;
        uint16_t sparkle_color = M5.Lcd.color565(255, 255, 255);
        
        sprite->drawLine(x-size, y, x+size, y, sparkle_color);
        sprite->drawLine(x, y-size, x, y+size, sparkle_color);
    }
    
    // Display FPS and spacecraft count
    sprite->setCursor(5, 5);
    sprite->setTextColor(TFT_WHITE, TFT_BLACK);
    sprite->printf("Ships: %d", spacecrafts.size());
}

void setup() {
    // Initialize M5Stack
    auto cfg = M5.config();
    M5.begin(cfg);
    
    // Setup display
    M5.Display.setRotation(1);  // Landscape mode
    M5.Display.fillScreen(BG_COLOR);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 10);
    M5.Display.println("Gravity Simulation");
    M5.Display.println("Tap to create spacecraft");
    M5.Display.println("Tap Earth to clear all");
    
    // Create sprite for double buffering
    sprite = new LGFX_Sprite(&M5.Display);
    sprite->setColorDepth(16);  // 16-bit color (RGB565)
    sprite->createSprite(WIDTH, HEIGHT);
    sprite->fillScreen(BG_COLOR);
    
    // Initialize
    moonAngle = 0;
    updateMoonPosition();
    
    delay(2000);  // Show instructions for 2 seconds
}

void loop() {
    static unsigned long lastFrameTime = 0;
    unsigned long currentTime = millis();
    
    // Target 30 FPS (33ms per frame)
    if (currentTime - lastFrameTime < 33) {
        return;
    }
    lastFrameTime = currentTime;
    
    M5.update();  // Update button and touch state
    
    // Handle touch input
    if (M5.Touch.getCount() > 0) {
        auto touch_point = M5.Touch.getDetail();
        if (touch_point.wasPressed()) {
            int x = touch_point.x;
            int y = touch_point.y;
            addSpacecraft(x, y);
        }
    }
    
    // Update moon position
    moonAngle += MOON_ANGULAR_VELOCITY;
    updateMoonPosition();
    
    // Update spacecraft
    for (int i = spacecrafts.size() - 1; i >= 0; i--) {
        // If update returns true (collision with Earth or Moon), remove spacecraft
        if (spacecrafts[i].update()) {
            spacecrafts.erase(spacecrafts.begin() + i);
        }
    }
    
    // Update explosion effects
    for (int i = explosionEffects.size() - 1; i >= 0; i--) {
        // Decrease effect lifetime
        explosionEffects[i].life -= 1;
        explosionEffects[i].alpha = 255 * (explosionEffects[i].life / 30.0);
        
        // Remove expired effects
        if (explosionEffects[i].life <= 0) {
            explosionEffects.erase(explosionEffects.begin() + i);
        }
    }
    
    // Update sparkle effects
    for (int i = sparkleEffects.size() - 1; i >= 0; i--) {
        // Decrease effect lifetime
        sparkleEffects[i].life -= 1;
        
        // Remove expired effects
        if (sparkleEffects[i].life <= 0) {
            sparkleEffects.erase(sparkleEffects.begin() + i);
        }
    }
    
    // Draw everything to sprite
    drawToSprite();
    
    // Push sprite to display (at once)
    sprite->pushSprite(0, 0);
}

void cleanup() {
    // Free sprite memory
    if (sprite != nullptr) {
        sprite->deleteSprite();
        delete sprite;
        sprite = nullptr;
    }
}


