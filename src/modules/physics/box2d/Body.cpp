/**
 * Copyright (c) 2006-2014 LOVE Development Team
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 **/

#include "Body.h"

#include "common/math.h"
#include "common/Memoizer.h"

#include "Shape.h"
#include "Fixture.h"
#include "World.h"
#include "Physics.h"

namespace love
{
namespace physics
{
namespace box2d
{

Body::Body(World *world, b2Vec2 p, Body::Type type)
	: world(world)
{
	world->retain();
	b2BodyDef def;
	def.position = Physics::scaleDown(p);
	body = world->world->CreateBody(&def);
	// Box2D body holds a reference to the love Body.
	this->retain();
	this->setType(type);
	Memoizer::add(body, this);
}

Body::Body(b2Body *b)
	: body(b)
{
	world = (World *)Memoizer::find(b->GetWorld());
	world->retain();
	// Box2D body holds a reference to the love Body.
	this->retain();
	Memoizer::add(body, this);
}

Body::~Body()
{
	world->release();
	body = 0;
}

float Body::getX()
{
	return Physics::scaleUp(body->GetPosition().x);
}

float Body::getY()
{
	return Physics::scaleUp(body->GetPosition().y);
}

void Body::getPosition(float &x_o, float &y_o)
{
	b2Vec2 v = Physics::scaleUp(body->GetPosition());
	x_o = v.x;
	y_o = v.y;
}

void Body::getLinearVelocity(float &x_o, float &y_o)
{
	b2Vec2 v = Physics::scaleUp(body->GetLinearVelocity());
	x_o = v.x;
	y_o = v.y;
}

float Body::getAngle()
{
	return body->GetAngle();
}

void Body::getWorldCenter(float &x_o, float &y_o)
{
	b2Vec2 v = Physics::scaleUp(body->GetWorldCenter());
	x_o = v.x;
	y_o = v.y;
}

void Body::getLocalCenter(float &x_o, float &y_o)
{
	b2Vec2 v = Physics::scaleUp(body->GetLocalCenter());
	x_o = v.x;
	y_o = v.y;
}

float Body::getAngularVelocity() const
{
	return body->GetAngularVelocity();
}

float Body::getMass() const
{
	return body->GetMass();
}

float Body::getInertia() const
{
	return Physics::scaleUp(Physics::scaleUp(body->GetInertia()));
}

int Body::getMassData(lua_State *L)
{
	b2MassData data;
	body->GetMassData(&data);
	b2Vec2 center = Physics::scaleUp(data.center);
	lua_pushnumber(L, center.x);
	lua_pushnumber(L, center.y);
	lua_pushnumber(L, data.mass);
	lua_pushnumber(L, Physics::scaleUp(Physics::scaleUp(data.I)));
	return 4;
}

float Body::getAngularDamping() const
{
	return body->GetAngularDamping();
}

float Body::getLinearDamping() const
{
	return body->GetLinearDamping();
}

float Body::getGravityScale() const
{
	return body->GetGravityScale();
}

Body::Type Body::getType() const
{
	switch (body->GetType())
	{
	case b2_staticBody:
		return BODY_STATIC;
		break;
	case b2_dynamicBody:
		return BODY_DYNAMIC;
		break;
	case b2_kinematicBody:
		return BODY_KINEMATIC;
		break;
	default:
		return BODY_INVALID;
		break;
	}
}

void Body::applyLinearImpulse(float jx, float jy, bool wake)
{
	body->ApplyLinearImpulse(Physics::scaleDown(b2Vec2(jx, jy)), body->GetWorldCenter(), wake);
}

void Body::applyLinearImpulse(float jx, float jy, float rx, float ry, bool wake)
{
	body->ApplyLinearImpulse(Physics::scaleDown(b2Vec2(jx, jy)), Physics::scaleDown(b2Vec2(rx, ry)), wake);
}

void Body::applyAngularImpulse(float impulse, bool wake)
{
	// Angular impulse is in kg*m^2/s, meaning it needs to be scaled twice
	body->ApplyAngularImpulse(Physics::scaleDown(Physics::scaleDown(impulse)), wake);
}

void Body::applyTorque(float t, bool wake)
{
	// Torque is in N*m, or kg*m^2/s^2, meaning it also needs to be scaled twice
	body->ApplyTorque(Physics::scaleDown(Physics::scaleDown(t)), wake);
}

void Body::applyForce(float fx, float fy, float rx, float ry, bool wake)
{
	body->ApplyForce(Physics::scaleDown(b2Vec2(fx, fy)), Physics::scaleDown(b2Vec2(rx, ry)), wake);
}

void Body::applyForce(float fx, float fy, bool wake)
{
	body->ApplyForceToCenter(Physics::scaleDown(b2Vec2(fx, fy)), wake);
}

void Body::setX(float x)
{
	body->SetTransform(Physics::scaleDown(b2Vec2(x, getY())), getAngle());
}

void Body::setY(float y)
{
	body->SetTransform(Physics::scaleDown(b2Vec2(getX(), y)), getAngle());
}

void Body::setLinearVelocity(float x, float y)
{
	body->SetLinearVelocity(Physics::scaleDown(b2Vec2(x, y)));
}

void Body::setAngle(float d)
{
	body->SetTransform(body->GetPosition(), d);
}

void Body::setAngularVelocity(float r)
{
	body->SetAngularVelocity(r);
}

void Body::setPosition(float x, float y)
{
	body->SetTransform(Physics::scaleDown(b2Vec2(x, y)), body->GetAngle());
}

void Body::setAngularDamping(float d)
{
	body->SetAngularDamping(d);
}

void Body::setLinearDamping(float d)
{
	body->SetLinearDamping(d);
}

void Body::resetMassData()
{
	body->ResetMassData();
}

void Body::setMassData(float x, float y, float m, float i)
{
	b2MassData massData;
	massData.center = Physics::scaleDown(b2Vec2(x, y));
	massData.mass = m;
	massData.I = Physics::scaleDown(Physics::scaleDown(i));
	body->SetMassData(&massData);
}

void Body::setMass(float m)
{
	b2MassData data;
	body->GetMassData(&data);
	data.mass = m;
	body->SetMassData(&data);
}

void Body::setInertia(float i)
{
	b2MassData massData;
	massData.center = body->GetLocalCenter();
	massData.mass = body->GetMass();
	massData.I = Physics::scaleDown(Physics::scaleDown(i));
	body->SetMassData(&massData);
}

void Body::setGravityScale(float scale)
{
	body->SetGravityScale(scale);
}

void Body::setType(Body::Type type)
{
	switch (type)
	{
	case Body::BODY_STATIC:
		body->SetType(b2_staticBody);
		break;
	case Body::BODY_DYNAMIC:
		body->SetType(b2_dynamicBody);
		break;
	case Body::BODY_KINEMATIC:
		body->SetType(b2_kinematicBody);
		break;
	default:
		break;
	}
}

void Body::getWorldPoint(float x, float y, float &x_o, float &y_o)
{
	b2Vec2 v = Physics::scaleUp(body->GetWorldPoint(Physics::scaleDown(b2Vec2(x, y))));
	x_o = v.x;
	y_o = v.y;
}

void Body::getWorldVector(float x, float y, float &x_o, float &y_o)
{
	b2Vec2 v = Physics::scaleUp(body->GetWorldVector(Physics::scaleDown(b2Vec2(x, y))));
	x_o = v.x;
	y_o = v.y;
}

int Body::getWorldPoints(lua_State *L)
{
	int argc = lua_gettop(L);
	int vcount = (int)argc/2;
	// at least one point
	love::luax_assert_argc(L, 2);

	for (int i = 0; i<vcount; i++)
	{
		float x = (float)lua_tonumber(L, 1);
		float y = (float)lua_tonumber(L, 2);
		// Remove them, so we don't run out of stack space
		lua_remove(L, 1);
		lua_remove(L, 1);
		// Time for scaling
		b2Vec2 point = Physics::scaleUp(body->GetWorldPoint(Physics::scaleDown(b2Vec2(x, y))));
		// And then we push the result
		lua_pushnumber(L, point.x);
		lua_pushnumber(L, point.y);
	}

	return argc;
}

void Body::getLocalPoint(float x, float y, float &x_o, float &y_o)
{
	b2Vec2 v = Physics::scaleUp(body->GetLocalPoint(Physics::scaleDown(b2Vec2(x, y))));
	x_o = v.x;
	y_o = v.y;
}

void Body::getLocalVector(float x, float y, float &x_o, float &y_o)
{
	b2Vec2 v = Physics::scaleUp(body->GetLocalVector(Physics::scaleDown(b2Vec2(x, y))));
	x_o = v.x;
	y_o = v.y;
}

void Body::getLinearVelocityFromWorldPoint(float x, float y, float &x_o, float &y_o)
{
	b2Vec2 v = Physics::scaleUp(body->GetLinearVelocityFromWorldPoint(Physics::scaleDown(b2Vec2(x, y))));
	x_o = v.x;
	y_o = v.y;
}

void Body::getLinearVelocityFromLocalPoint(float x, float y, float &x_o, float &y_o)
{
	b2Vec2 v = Physics::scaleUp(body->GetLinearVelocityFromLocalPoint(Physics::scaleDown(b2Vec2(x, y))));
	x_o = v.x;
	y_o = v.y;
}

bool Body::isBullet() const
{
	return body->IsBullet();
}

void Body::setBullet(bool bullet)
{
	return body->SetBullet(bullet);
}

bool Body::isActive() const
{
	return body->IsActive();
}

bool Body::isAwake() const
{
	return body->IsAwake();
}

void Body::setSleepingAllowed(bool allow)
{
	body->SetSleepingAllowed(allow);
}

bool Body::isSleepingAllowed() const
{
	return body->IsSleepingAllowed();
}

void Body::setActive(bool active)
{
	body->SetActive(active);
}

void Body::setAwake(bool awake)
{
	body->SetAwake(awake);
}

void Body::setFixedRotation(bool fixed)
{
	body->SetFixedRotation(fixed);
}

bool Body::isFixedRotation() const
{
	return body->IsFixedRotation();
}

World *Body::getWorld() const
{
	return world;
}

int Body::getFixtureList(lua_State *L) const
{
	lua_newtable(L);
	b2Fixture *f = body->GetFixtureList();
	int i = 1;
	do
	{
		if (!f)
			break;
		Fixture *fixture = (Fixture *)Memoizer::find(f);
		if (!fixture)
			throw love::Exception("A fixture has escaped Memoizer!");
		fixture->retain();
		luax_pushtype(L, "Fixture", PHYSICS_FIXTURE_T, fixture);
		lua_rawseti(L, -2, i);
		i++;
	}
	while ((f = f->GetNext()));
	return 1;
}

b2Vec2 Body::getVector(lua_State *L)
{
	love::luax_assert_argc(L, 2, 2);
	b2Vec2 v((float)lua_tonumber(L, 1), (float)lua_tonumber(L, 2));
	lua_pop(L, 2);
	return v;
}

int Body::pushVector(lua_State *L, const b2Vec2 &v)
{
	lua_pushnumber(L, v.x);
	lua_pushnumber(L, v.y);
	return 2;
}

void Body::destroy()
{
	if (world->world->IsLocked())
	{
		// Called during time step. Save reference for destruction afterwards.
		this->retain();
		world->destructBodies.push_back(this);
		return;
	}

	world->world->DestroyBody(body);
	Memoizer::remove(body);
	body = NULL;

	// Box2D body destroyed. Release its reference to the love Body.
	this->release();
}

struct ShapeCastResults
{
	ShapeCastResults() : fraction( 1.0f ), hitBody( NULL )
	{
	}
	float32 fraction;
	b2Vec2 hitPoint; //The position where the shape is stopped.
	b2Vec2 myContactPoint; // The point-of-contact of the cast. Do not confuse with hitPos.
	b2Vec2 otherContactPoint;
	b2Vec2 contactNormal;
	b2Body* hitBody;
};

void GetSweepInfo( b2Body* body, const b2Vec2 p1, const b2Vec2& p2, b2Sweep* out )
{

	out->localCenter.SetZero();
	out->c0 = p1;
	out->c = p2;
	out->a0 = body->GetAngle();
	out->a = out->a0;
}

void GetObstacleSweepInfo( b2Body* body, b2Sweep* out )
{
	out->localCenter.SetZero();
	out->c0 = body->GetPosition();
	out->c = out->c0;
	out->a0 = body->GetAngle();
	out->a = out->a0;
}

void SweepShape( b2TOIInput* input, b2Body* bodyA, b2Body* bodyB, ShapeCastResults* out )
{
	// "Part A" of the input should be considered constant. Fill out "Part B".
	input->proxyB.Set( bodyB->GetFixtureList()->GetShape(), 0 );
	GetObstacleSweepInfo( bodyB, &input->sweepB );

	// Do the actual sweep.
	b2TOIOutput output;

	b2TimeOfImpact( &output, input );

	if ( output.state == b2TOIOutput::e_touching && output.t < 1.0f )
	{
		// Get contact points and contact normal.
		b2SimplexCache cache;
		cache.count = 0;

		b2DistanceOutput distanceOutput;
		b2DistanceInput distanceInput;

		distanceInput.proxyA = input->proxyA;
		distanceInput.proxyB = input->proxyB;

		input->sweepA.GetTransform( &distanceInput.transformA, output.t );
		input->sweepB.GetTransform( &distanceInput.transformB, output.t );
		distanceInput.useRadii = false;



		b2Distance( &distanceOutput, &cache, &distanceInput );

		// Fill out the results.
		out->fraction = output.t;
		out->hitPoint = distanceInput.transformA.p;
		out->myContactPoint = distanceOutput.pointA;
		out->otherContactPoint = distanceOutput.pointB;
		out->contactNormal = b2Vec2( distanceOutput.pointB.x - distanceOutput.pointA.x, distanceOutput.pointB.y - distanceOutput.pointA.y );
		out->contactNormal.Normalize();
		out->hitBody = bodyB;
		return;
	}
	// Indicate that no contact was made. The rest of the values will be garbage but that's fine.
	out->hitBody = NULL;
}

class BodyCastCallback : public b2QueryCallback
{
public:
	BodyCastCallback( lua_State* L, b2Body* body, b2TOIInput* traceInfo ) :
	  L(L),
		  traceInfo( traceInfo ),
		  body( body )
	  {
	  }

private:
	b2TOIInput* traceInfo;
	b2Body* body;
	lua_State* L;

public:
	/// Called for each fixture found in the query AABB.
	/// @return false to terminate the query.
	virtual bool ReportFixture( b2Fixture* testFixture ) // If this is getting called for every fixture, it will be redundant. The actual shape cast tests entire shapes, not single fixtures.
	{
		b2Body* otherBody = testFixture->GetBody();

		// Do the sweep.
		ShapeCastResults thisTestResult;
		SweepShape( traceInfo, body, otherBody, &thisTestResult );

		if ( !thisTestResult.hitBody )
			return true;

		lua_pushvalue( L, -1 ); // Push the callback.

		Fixture *fixture = (Fixture *)Memoizer::find(testFixture);
		if (!fixture)
			throw love::Exception("A fixture has escaped Memoizer!");
		fixture->retain();

		// fixture ( fixture )
		luax_pushtype( L, "Fixture", PHYSICS_FIXTURE_T, fixture );

		// fraction ( fraction )
		lua_pushnumber( L, thisTestResult.fraction );

		// hitPoint ( x, y )
		b2Vec2 outHitPoint = Physics::scaleUp( thisTestResult.hitPoint );
		lua_pushnumber( L, outHitPoint.x );
		lua_pushnumber( L, outHitPoint.y );

		// contactNormal ( nx, ny )
		lua_pushnumber( L, thisTestResult.contactNormal.x );
		lua_pushnumber( L, thisTestResult.contactNormal.y );

		// myContactPoint ( mx, my )
		b2Vec2 outMyContactPoint = Physics::scaleUp( thisTestResult.myContactPoint );
		lua_pushnumber( L, outMyContactPoint.x );
		lua_pushnumber( L, outMyContactPoint.y );

		// otherContactPoint ( ox, oy )
		b2Vec2 outOtherContactPoint = Physics::scaleUp( thisTestResult.otherContactPoint );
		lua_pushnumber( L, outOtherContactPoint.x );
		lua_pushnumber( L, outOtherContactPoint.y );

		lua_call( L, 10, 1 );

		bool shouldContinue = lua_isnil( L, -1 ) || lua_toboolean( L, -1 );
		lua_pop( L, 1 );

		return shouldContinue;
	}
};

int Body::sweep( lua_State *L )
{

	luax_assert_argc(L, 5);
	lua_settop( L, 5 );

	float x1 = (float)luaL_checknumber(L, 1);
	float y1 = (float)luaL_checknumber(L, 2);
	float x2 = (float)luaL_checknumber(L, 3);
	float y2 = (float)luaL_checknumber(L, 4);
	b2Vec2 v1 = Physics::scaleDown(b2Vec2(x1, y1));
	b2Vec2 v2 = Physics::scaleDown(b2Vec2(x2, y2));

	// Fill out the half of the TOI input struct that's going to be constant.
	b2TOIInput traceInfo;
	traceInfo.tMax = 1.0f;
	traceInfo.proxyA.Set( body->GetFixtureList()->GetShape(), 0 );
	GetSweepInfo( body, v1, v2, &(traceInfo.sweepA) );

	// Figure out out bbox.
	b2AABB bbox;
	b2Transform transforms = body->GetTransform();
	transforms.p.SetZero();
	body->GetFixtureList()->GetShape()->ComputeAABB( &bbox, transforms, 0 );

	b2Vec2 bboxSize = b2Vec2( bbox.upperBound.x - bbox.lowerBound.x, bbox.upperBound.y - bbox.lowerBound.y );

	// Figure out the bbox of this cast.
	b2AABB bboxStart = bbox;
	//TranslateAABB( &bboxStart, src );
	bboxStart.lowerBound += v1;
	bboxStart.upperBound += v1;
	b2AABB bboxEnd = bbox;
	//TranslateAABB( &bboxEnd, dest );
	bboxEnd.lowerBound += v2;
	bboxEnd.upperBound += v2;

	b2AABB testBox;
	testBox.Combine( bboxStart, bboxEnd );

	// Cast this body against all others in the cast bbox.

	BodyCastCallback callback( L, body, &traceInfo );
	world->world->QueryAABB( &callback, testBox );

	return 0;
}

} // box2d
} // physics
} // love
