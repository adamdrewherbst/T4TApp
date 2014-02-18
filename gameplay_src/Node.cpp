#include "Base.h"
#include "Node.h"
#include "AudioSource.h"
#include "Scene.h"
#include "Joint.h"
#include "PhysicsRigidBody.h"
#include "PhysicsVehicle.h"
#include "PhysicsVehicleWheel.h"
#include "PhysicsGhostObject.h"
#include "PhysicsCharacter.h"
#include "Game.h"
#include "Terrain.h"

// Node dirty flags
#define NODE_DIRTY_WORLD 1
#define NODE_DIRTY_BOUNDS 2
#define NODE_DIRTY_ALL (NODE_DIRTY_WORLD | NODE_DIRTY_BOUNDS)

namespace gameplay
{

Node::Node(const char* id)
    : _scene(NULL), _firstChild(NULL), _nextSibling(NULL), _prevSibling(NULL), _parent(NULL), _childCount(0),
    _tags(NULL), _camera(NULL), _light(NULL), _model(NULL), _terrain(NULL), _form(NULL), _audioSource(NULL), _particleEmitter(NULL),
    _collisionObject(NULL), _agent(NULL), _dirtyBits(NODE_DIRTY_ALL), _notifyHierarchyChanged(true), _userData(NULL)
{
    if (id)
    {
        _id = id;
    }
}

Node::~Node()
{
    removeAllChildren();

    if (_model)
        _model->setNode(NULL);
    if (_audioSource)
        _audioSource->setNode(NULL);
    if (_particleEmitter)
        _particleEmitter->setNode(NULL);
    if (_form)
        _form->setNode(NULL);

    SAFE_RELEASE(_camera);
    SAFE_RELEASE(_light);
    SAFE_RELEASE(_model);
    SAFE_RELEASE(_terrain);
    SAFE_RELEASE(_audioSource);
    SAFE_RELEASE(_particleEmitter);
    SAFE_RELEASE(_form);
    SAFE_DELETE(_collisionObject);
    SAFE_DELETE(_tags);

    setAgent(NULL);

    // Cleanup user data
    if (_userData)
    {
        // Call custom cleanup callback if specified
        if (_userData->cleanupCallback)
            _userData->cleanupCallback(_userData->pointer);
        SAFE_DELETE(_userData);
    }
}

Node* Node::create(const char* id)
{
    return new Node(id);
}

const char* Node::getId() const
{
    return _id.c_str();
}

void Node::setId(const char* id)
{
    if (id)
    {
        _id = id;
    }
}

Node::Type Node::getType() const
{
    return Node::NODE;
}

void Node::addChild(Node* child)
{
    GP_ASSERT(child);

    if (child->_parent == this)
    {
        // This node is already present in our hierarchy
        return;
    }

    child->addRef();

    // If the item belongs to another hierarchy, remove it first.
    if (child->_parent)
    {
        child->_parent->removeChild(child);
    }
    else if (child->_scene)
    {
        child->_scene->removeNode(child);
    }

    // Order is irrelevant, so add to the beginning of the list.
    if (_firstChild)
    {
        _firstChild->_prevSibling = child;
        child->_nextSibling = _firstChild;
        _firstChild = child;
    }
    else
    {
        _firstChild = child;
    }

    child->_parent = this;

    ++_childCount;

    setBoundsDirty();

    if (_notifyHierarchyChanged)
    {
        hierarchyChanged();
    }
}

void Node::removeChild(Node* child)
{
    if (child == NULL || child->_parent != this)
    {
        // The child is not in our hierarchy.
        return;
    }

    // Call remove on the child.
    child->remove();

    SAFE_RELEASE(child);
}

void Node::removeAllChildren()
{
    _notifyHierarchyChanged = false;

    while (_firstChild)
    {
        removeChild(_firstChild);
    }

    _notifyHierarchyChanged = true;
    hierarchyChanged();
}

void Node::remove()
{
    // Re-link our neighbours.
    if (_prevSibling)
    {
        _prevSibling->_nextSibling = _nextSibling;
    }
    if (_nextSibling)
    {
        _nextSibling->_prevSibling = _prevSibling;
    }

    // Update our parent.
    Node* parent = _parent;
    if (parent)
    {
        if (this == parent->_firstChild)
        {
            parent->_firstChild = _nextSibling;
        }

        --parent->_childCount;
    }

    _nextSibling = NULL;
    _prevSibling = NULL;
    _parent = NULL;

    if (parent && parent->_notifyHierarchyChanged)
    {
        parent->hierarchyChanged();
    }
}

Node* Node::getFirstChild() const
{
    return _firstChild;
}

Node* Node::getNextSibling() const
{
    return _nextSibling;
}

Node* Node::getPreviousSibling() const
{
    return _prevSibling;
}

Node* Node::getParent() const
{
    return _parent;
}

bool Node::hasTag(const char* name) const
{
    GP_ASSERT(name);

    return (_tags ? _tags->find(name) != _tags->end() : false);
}

const char* Node::getTag(const char* name) const
{
    GP_ASSERT(name);

    if (!_tags)
        return NULL;

    std::map<std::string, std::string>::const_iterator itr = _tags->find(name);
    return (itr == _tags->end() ? NULL : itr->second.c_str());
}

void Node::setTag(const char* name, const char* value)
{
    GP_ASSERT(name);

    if (value == NULL)
    {
        // Removing tag
        if (_tags)
        {
            _tags->erase(name);
            if (_tags->size() == 0)
                SAFE_DELETE(_tags);
        }
    }
    else
    {
        // Setting tag
        if (_tags == NULL)
            _tags = new std::map<std::string, std::string>();

        (*_tags)[name] = value;
    }
}

void* Node::getUserPointer() const
{
    return (_userData ? _userData->pointer : NULL);
}

void Node::setUserPointer(void* pointer, void (*cleanupCallback)(void*))
{
    // If existing user pointer is being changed, call cleanup function to free previous pointer
    if (_userData && _userData->pointer && _userData->cleanupCallback && pointer != _userData->pointer)
    {
        _userData->cleanupCallback(_userData->pointer);
    }

    if (pointer)
    {
        // Assign user pointer
        if (_userData == NULL)
            _userData = new UserData();

        _userData->pointer = pointer;
        _userData->cleanupCallback = cleanupCallback;
    }
    else
    {
        // Clear user pointer
        SAFE_DELETE(_userData);
    }
}

unsigned int Node::getChildCount() const
{
    return _childCount;
}

Node* Node::findNode(const char* id, bool recursive, bool exactMatch) const
{
    GP_ASSERT(id);

    // If the node has a model with a mesh skin, search the skin's hierarchy as well.
    Node* rootNode = NULL;
    if (_model != NULL && _model->getSkin() != NULL && (rootNode = _model->getSkin()->_rootNode) != NULL)
    {
        if ((exactMatch && rootNode->_id == id) || (!exactMatch && rootNode->_id.find(id) == 0))
            return rootNode;
        
        Node* match = rootNode->findNode(id, true, exactMatch);
        if (match)
        {
            return match;
        }
    }
    
    // Search immediate children first.
    for (Node* child = getFirstChild(); child != NULL; child = child->getNextSibling())
    {
        // Does this child's ID match?
        if ((exactMatch && child->_id == id) || (!exactMatch && child->_id.find(id) == 0))
        {
            return child;
        }
    }

    // Recurse.
    if (recursive)
    {
        for (Node* child = getFirstChild(); child != NULL; child = child->getNextSibling())
        {
            Node* match = child->findNode(id, true, exactMatch);
            if (match)
            {
                return match;
            }
        }
    }

    return NULL;
}   

unsigned int Node::findNodes(const char* id, std::vector<Node*>& nodes, bool recursive, bool exactMatch) const
{
    GP_ASSERT(id);
    
    unsigned int count = 0;

    // If the node has a model with a mesh skin, search the skin's hierarchy as well.
    Node* rootNode = NULL;
    if (_model != NULL && _model->getSkin() != NULL && (rootNode = _model->getSkin()->_rootNode) != NULL)
    {
        if ((exactMatch && rootNode->_id == id) || (!exactMatch && rootNode->_id.find(id) == 0))
        {
            nodes.push_back(rootNode);
            ++count;
        }
        count += rootNode->findNodes(id, nodes, true, exactMatch);
    }

    // Search immediate children first.
    for (Node* child = getFirstChild(); child != NULL; child = child->getNextSibling())
    {
        // Does this child's ID match?
        if ((exactMatch && child->_id == id) || (!exactMatch && child->_id.find(id) == 0))
        {
            nodes.push_back(child);
            ++count;
        }
    }

    // Recurse.
    if (recursive)
    {
        for (Node* child = getFirstChild(); child != NULL; child = child->getNextSibling())
        {
            count += child->findNodes(id, nodes, true, exactMatch);
        }
    }

    return count;
}

Scene* Node::getScene() const
{
    if (_scene)
        return _scene;

    // Search our parent for the scene
    if (_parent)
    {
        Scene* scene = _parent->getScene();
        if (scene)
            return scene;
    }

    return NULL;
}

Node* Node::getRootNode() const
{
    Node* n = const_cast<Node*>(this);
    while (n->getParent())
    {
        n = n->getParent();
    }
    return n;
}

bool Node::isStatic() const
{
    return (_collisionObject && _collisionObject->isStatic());
}

const Matrix& Node::getWorldMatrix() const
{
    if (_dirtyBits & NODE_DIRTY_WORLD)
    {
        // Clear our dirty flag immediately to prevent this block from being entered if our
        // parent calls our getWorldMatrix() method as a result of the following calculations.
        _dirtyBits &= ~NODE_DIRTY_WORLD;

        if (!isStatic())
        {
            // If we have a parent, multiply our parent world transform by our local
            // transform to obtain our final resolved world transform.
            Node* parent = getParent();
            if (parent && (!_collisionObject || _collisionObject->isKinematic()))
            {
                Matrix::multiply(parent->getWorldMatrix(), getMatrix(), &_world);
            }
            else
            {
                _world = getMatrix();
            }

            // Our world matrix was just updated, so call getWorldMatrix() on all child nodes
            // to force their resolved world matrices to be updated.
            for (Node* child = getFirstChild(); child != NULL; child = child->getNextSibling())
            {
                child->getWorldMatrix();
            }
        }
    }

    return _world;
}

const Matrix& Node::getWorldViewMatrix() const
{
    static Matrix worldView;

    Matrix::multiply(getViewMatrix(), getWorldMatrix(), &worldView);

    return worldView;
}

const Matrix& Node::getInverseTransposeWorldViewMatrix() const
{
    static Matrix invTransWorldView;
    Matrix::multiply(getViewMatrix(), getWorldMatrix(), &invTransWorldView);
    invTransWorldView.invert();
    invTransWorldView.transpose();
    return invTransWorldView;
}

const Matrix& Node::getInverseTransposeWorldMatrix() const
{
    static Matrix invTransWorld;
    invTransWorld = getWorldMatrix();
    invTransWorld.invert();
    invTransWorld.transpose();
    return invTransWorld;
}

const Matrix& Node::getViewMatrix() const
{
    Scene* scene = getScene();
    Camera* camera = scene ? scene->getActiveCamera() : NULL;
    if (camera)
    {
        return camera->getViewMatrix();
    }
    else
    {
        return Matrix::identity();
    }
}

const Matrix& Node::getInverseViewMatrix() const
{
    Scene* scene = getScene();
    Camera* camera = scene ? scene->getActiveCamera() : NULL;
    if (camera)
    {
        return camera->getInverseViewMatrix();
    }
    else
    {
        return Matrix::identity();
    }
}

const Matrix& Node::getProjectionMatrix() const
{
    Scene* scene = getScene();
    Camera* camera = scene ? scene->getActiveCamera() : NULL;
    if (camera)
    {
        return camera->getProjectionMatrix();
    }
    else
    {
        return Matrix::identity();
    }
}

const Matrix& Node::getViewProjectionMatrix() const
{
    Scene* scene = getScene();
    Camera* camera = scene ? scene->getActiveCamera() : NULL;
    if (camera)
    {
        return camera->getViewProjectionMatrix();
    }
    else
    {
        return Matrix::identity();
    }
}

const Matrix& Node::getInverseViewProjectionMatrix() const
{
    Scene* scene = getScene();
    Camera* camera = scene ? scene->getActiveCamera() : NULL;
    if (camera)
    {
        return camera->getInverseViewProjectionMatrix();
    }

    return Matrix::identity();
}

const Matrix& Node::getWorldViewProjectionMatrix() const
{
    static Matrix worldViewProj;

    // Always re-calculate worldViewProjection matrix since it's extremely difficult
    // to track whether the camera has changed (it may frequently change every frame).
    Matrix::multiply(getViewProjectionMatrix(), getWorldMatrix(), &worldViewProj);

    return worldViewProj;
}

Vector3 Node::getTranslationWorld() const
{
    Vector3 translation;
    getWorldMatrix().getTranslation(&translation);
    return translation;
}

Vector3 Node::getTranslationView() const
{
    Vector3 translation;
    getWorldMatrix().getTranslation(&translation);
    getViewMatrix().transformPoint(&translation);
    return translation;
}

Vector3 Node::getForwardVectorWorld() const
{
    Vector3 vector;
    getWorldMatrix().getForwardVector(&vector);
    return vector;
}

Vector3 Node::getForwardVectorView() const
{
    Vector3 vector;
    getWorldMatrix().getForwardVector(&vector);
    getViewMatrix().transformVector(&vector);
    return vector;
}

Vector3 Node::getRightVectorWorld() const
{
    Vector3 vector;
    getWorldMatrix().getRightVector(&vector);
    return vector;
}

Vector3 Node::getUpVectorWorld() const
{
    Vector3 vector;
    getWorldMatrix().getUpVector(&vector);
    return vector;
}

Vector3 Node::getActiveCameraTranslationWorld() const
{
    Scene* scene = getScene();
    if (scene)
    {
        Camera* camera = scene->getActiveCamera();
        if (camera)
        {
            Node* cameraNode = camera->getNode();
            if (cameraNode)
            {
                return cameraNode->getTranslationWorld();
            }
        }
    }

    return Vector3::zero();
}

Vector3 Node::getActiveCameraTranslationView() const
{
    Scene* scene = getScene();
    if (scene)
    {
        Camera* camera = scene->getActiveCamera();
        if (camera)
        {
            Node* cameraNode = camera->getNode();
            if (cameraNode)
            {
                return cameraNode->getTranslationView();
            }
        }
    }

    return Vector3::zero();
}

void Node::hierarchyChanged()
{
    // When our hierarchy changes our world transform is affected, so we must dirty it.
    transformChanged();
}

void Node::transformChanged()
{
    // Our local transform was changed, so mark our world matrices dirty.
    _dirtyBits |= NODE_DIRTY_WORLD | NODE_DIRTY_BOUNDS;

    // Notify our children that their transform has also changed (since transforms are inherited).
    for (Node* n = getFirstChild(); n != NULL; n = n->getNextSibling())
    {
        if (Transform::isTransformChangedSuspended())
        {
            // If the DIRTY_NOTIFY bit is not set
            if (!n->isDirty(Transform::DIRTY_NOTIFY))
            {
                n->transformChanged();
                suspendTransformChange(n);
            }
        }
        else
        {
            n->transformChanged();
        }
    }

    Transform::transformChanged();
}

void Node::setBoundsDirty()
{
    // Mark ourself and our parent nodes as dirty
    _dirtyBits |= NODE_DIRTY_BOUNDS;

    // Mark our parent bounds as dirty as well
    if (_parent)
        _parent->setBoundsDirty();
}

Animation* Node::getAnimation(const char* id) const
{
    Animation* animation = ((AnimationTarget*)this)->getAnimation(id);
    if (animation)
        return animation;
    
    // See if this node has a model, then drill down.
    Model* model = this->getModel();
    if (model)
    {
        // Check to see if there's any animations with the ID on the joints.
        MeshSkin* skin = model->getSkin();
        if (skin)
        {
            Node* rootNode = skin->_rootNode;
            if (rootNode)
            {
                animation = rootNode->getAnimation(id);
                if (animation)
                    return animation;
            }
        }

        // Check to see if any of the model's material parameter's has an animation
        // with the given ID.
        Material* material = model->getMaterial();
        if (material)
        {
            // How to access material parameters? hidden on the Material::RenderState.
            std::vector<MaterialParameter*>::iterator itr = material->_parameters.begin();
            for (; itr != material->_parameters.end(); itr++)
            {
                GP_ASSERT(*itr);
                animation = ((MaterialParameter*)(*itr))->getAnimation(id);
                if (animation)
                    return animation;
            }
        }
    }

    // look through form for animations.
    Form* form = this->getForm();
    if (form)
    {
        animation = form->getAnimation(id);
        if (animation)
            return animation;
    }

    // Look through this node's children for an animation with the specified ID.
    for (Node* child = getFirstChild(); child != NULL; child = child->getNextSibling())
    {
        animation = child->getAnimation(id);
        if (animation)
            return animation;
    }
    
    return NULL;
}

Camera* Node::getCamera() const
{
    return _camera;
}

void Node::setCamera(Camera* camera)
{
    if (_camera != camera)
    {
        if (_camera)
        {
            _camera->setNode(NULL);
            SAFE_RELEASE(_camera);
        }

        _camera = camera;

        if (_camera)
        {
            _camera->addRef();
            _camera->setNode(this);
        }
    }
}

Light* Node::getLight() const
{
    return _light;
}

void Node::setLight(Light* light)
{
    if (_light != light)
    {
        if (_light)
        {
            _light->setNode(NULL);
            SAFE_RELEASE(_light);
        }

        _light = light;

        if (_light)
        {
            _light->addRef();
            _light->setNode(this);
        }

        setBoundsDirty();
    }
}

Model* Node::getModel() const
{
    return _model;
}

void Node::setModel(Model* model)
{
    if (_model != model)
    {
        if (_model)
        {
            _model->setNode(NULL);
            SAFE_RELEASE(_model);
        }

        _model = model;

        if (_model)
        {
            _model->addRef();
            _model->setNode(this);
        }
    }
}

Terrain* Node::getTerrain() const
{
    return _terrain;
}

void Node::setTerrain(Terrain* terrain)
{
    if (_terrain != terrain)
    {
        if (_terrain)
        {
            _terrain->setNode(NULL);
            SAFE_RELEASE(_terrain);
        }

        _terrain = terrain;

        if (_terrain)
        {
            _terrain->addRef();
            _terrain->setNode(this);
        }

        setBoundsDirty();
    }
}

Form* Node::getForm() const
{
    return _form;
}

void Node::setForm(Form* form)
{
    if (_form != form)
    {
        if (_form)
        {
            _form->setNode(NULL);
            SAFE_RELEASE(_form);
        }

        _form = form;

        if (_form)
        {
            _form->addRef();
            _form->setNode(this);
        }
    }
}

const BoundingSphere& Node::getBoundingSphere() const
{
    if (_dirtyBits & NODE_DIRTY_BOUNDS)
    {
        _dirtyBits &= ~NODE_DIRTY_BOUNDS;

        const Matrix& worldMatrix = getWorldMatrix();

        // Start with our local bounding sphere
        // TODO: Incorporate bounds from entities other than mesh (i.e. emitters, audiosource, etc)
        bool empty = true;
        if (_terrain)
        {
            _bounds.set(_terrain->getBoundingBox());
            empty = false;
        }
        if (_model && _model->getMesh())
        {
            if (empty)
            {
                _bounds.set(_model->getMesh()->getBoundingSphere());
                empty = false;
            }
            else
            {
                _bounds.merge(_model->getMesh()->getBoundingSphere());
            }
        }
        if (_light)
        {
            switch (_light->getLightType())
            {
            case Light::POINT:
                if (empty)
                {
                    _bounds.set(Vector3::zero(), _light->getRange());
                    empty = false;
                }
                else
                {
                    _bounds.merge(BoundingSphere(Vector3::zero(), _light->getRange()));
                }
                break;
            case Light::SPOT:
                // TODO: Implement spot light bounds
                break;
            }
        }
        if (empty)
        {
            // Empty bounding sphere, set the world translation with zero radius
            worldMatrix.getTranslation(&_bounds.center);
            _bounds.radius = 0;
        }

        // Transform the sphere (if not empty) into world space.
        if (!empty)
        {
            bool applyWorldTransform = true;
            if (_model && _model->getSkin())
            {
                // Special case: If the root joint of our mesh skin is parented by any nodes, 
                // multiply the world matrix of the root joint's parent by this node's
                // world matrix. This computes a final world matrix used for transforming this
                // node's bounding volume. This allows us to store a much smaller bounding
                // volume approximation than would otherwise be possible for skinned meshes,
                // since joint parent nodes that are not in the matrix palette do not need to
                // be considered as directly transforming vertices on the GPU (they can instead
                // be applied directly to the bounding volume transformation below).
                GP_ASSERT(_model->getSkin()->getRootJoint());
                Node* jointParent = _model->getSkin()->getRootJoint()->getParent();
                if (jointParent)
                {
                    // TODO: Should we protect against the case where joints are nested directly
                    // in the node hierachy of the model (this is normally not the case)?
                    Matrix boundsMatrix;
                    Matrix::multiply(getWorldMatrix(), jointParent->getWorldMatrix(), &boundsMatrix);
                    _bounds.transform(boundsMatrix);
                    applyWorldTransform = false;
                }
            }
            if (applyWorldTransform)
            {
                _bounds.transform(getWorldMatrix());
            }
        }

        // Merge this world-space bounding sphere with our childrens' bounding volumes.
        for (Node* n = getFirstChild(); n != NULL; n = n->getNextSibling())
        {
            const BoundingSphere& childSphere = n->getBoundingSphere();
            if (!childSphere.isEmpty())
            {
                if (empty)
                {
                    _bounds.set(childSphere);
                    empty = false;
                }
                else
                {
                    _bounds.merge(childSphere);
                }
            }
        }
    }

    return _bounds;
}

Node* Node::clone() const
{
    NodeCloneContext context;
    return cloneRecursive(context);
}

Node* Node::cloneSingleNode(NodeCloneContext &context) const
{
    Node* copy = Node::create(getId());
    context.registerClonedNode(this, copy);
    cloneInto(copy, context);
    return copy;
}

Node* Node::cloneRecursive(NodeCloneContext &context) const
{
    Node* copy = cloneSingleNode(context);
    GP_ASSERT(copy);

    // Find our current last child
    Node* lastChild = NULL;
    for (Node* child = getFirstChild(); child != NULL; child = child->getNextSibling())
    {
        lastChild = child;
    }

    // Loop through the nodes backwards because addChild adds the node to the front.
    for (Node* child = lastChild; child != NULL; child = child->getPreviousSibling())
    {
        Node* childCopy = child->cloneRecursive(context);
        GP_ASSERT(childCopy);
        copy->addChild(childCopy);
        childCopy->release();
    }

    return copy;
}

void Node::cloneInto(Node* node, NodeCloneContext &context) const
{
    GP_ASSERT(node);
    Transform::cloneInto(node, context);

    // TODO: Clone the rest of the node data.

    if (Camera* camera = getCamera())
    {
        Camera* cameraClone = camera->clone(context);
        node->setCamera(cameraClone);
        cameraClone->release();
    }
    if (Light* light = getLight())
    {
        Light* lightClone = light->clone(context);
        node->setLight(lightClone);
        lightClone->release();
    }
    if (AudioSource* audio = getAudioSource())
    {
        AudioSource* audioClone = audio->clone(context);
        node->setAudioSource(audioClone);
        audioClone->release();
    }
    if (Model* model = getModel())
    {
        Model* modelClone = model->clone(context);
        node->setModel(modelClone);
        modelClone->release();
    }
    node->_world = _world;
    node->_bounds = _bounds;

    // Note: Do not clone _userData - we can't make any assumptions about its content and how it's managed,
    // so it's the caller's responsibility to clone user data if needed.

    if (_tags)
    {
        node->_tags = new std::map<std::string, std::string>(_tags->begin(), _tags->end());
    }
}

AudioSource* Node::getAudioSource() const
{
    return _audioSource;
}

void Node::setAudioSource(AudioSource* audio)
{
    if (_audioSource != audio)
    {
        if (_audioSource)
        {
            _audioSource->setNode(NULL);
            SAFE_RELEASE(_audioSource);
        }
        
        _audioSource = audio;

        if (_audioSource)
        {
            _audioSource->addRef();
            _audioSource->setNode(this);
        }
    }
}

ParticleEmitter* Node::getParticleEmitter() const
{
    return _particleEmitter;
}

void Node::setParticleEmitter(ParticleEmitter* emitter)
{
    if (_particleEmitter != emitter)
    {
        if (_particleEmitter)
        {
            _particleEmitter->setNode(NULL);
            SAFE_RELEASE(_particleEmitter);
        }
        
        _particleEmitter = emitter;

        if (_particleEmitter)
        {
            _particleEmitter->addRef();
            _particleEmitter->setNode(this);
        }
    }
}

PhysicsCollisionObject* Node::getCollisionObject() const
{
    return _collisionObject;
}

PhysicsCollisionObject* Node::setCollisionObject(PhysicsCollisionObject::Type type, const PhysicsCollisionShape::Definition& shape, PhysicsRigidBody::Parameters* rigidBodyParameters)
{
    SAFE_DELETE(_collisionObject);

    switch (type)
    {
    case PhysicsCollisionObject::RIGID_BODY:
        {
            _collisionObject = new PhysicsRigidBody(this, shape, rigidBodyParameters ? *rigidBodyParameters : PhysicsRigidBody::Parameters());
        }
        break;

    case PhysicsCollisionObject::GHOST_OBJECT:
        {
            _collisionObject = new PhysicsGhostObject(this, shape);
        }
        break;

    case PhysicsCollisionObject::CHARACTER:
        {
            _collisionObject = new PhysicsCharacter(this, shape, rigidBodyParameters ? rigidBodyParameters->mass : 1.0f);
        }
        break;

    case PhysicsCollisionObject::VEHICLE:
        {
            _collisionObject = new PhysicsVehicle(this, shape, rigidBodyParameters ? *rigidBodyParameters : PhysicsRigidBody::Parameters());
        }
        break;

    case PhysicsCollisionObject::VEHICLE_WHEEL:
        {
            //
            // PhysicsVehicleWheel is special because this call will traverse up the scene graph for the
            // first ancestor node that is shared with another node of collision type VEHICLE, and then
            // proceed to add itself as a wheel onto that vehicle. This is by design, and allows the
            // visual scene hierarchy to be the sole representation of the relationship between physics
            // objects rather than forcing that upon the otherwise-flat ".physics" (properties) file.
            //
            // IMPORTANT: The VEHICLE must come before the VEHICLE_WHEEL in the ".scene" (properties) file!
            //
            _collisionObject = new PhysicsVehicleWheel(this, shape, rigidBodyParameters ? *rigidBodyParameters : PhysicsRigidBody::Parameters());
        }
        break;

    case PhysicsCollisionObject::NONE:
        break;  // Already deleted, Just don't add a new collision object back.
    }

    return _collisionObject;
}

PhysicsCollisionObject* Node::setCollisionObject(const char* url)
{
    // Load the collision object properties from file.
    Properties* properties = Properties::create(url);
    if (properties == NULL)
    {
        GP_ERROR("Failed to load collision object file: %s", url);
        return NULL;
    }

    PhysicsCollisionObject* collisionObject = setCollisionObject((strlen(properties->getNamespace()) > 0) ? properties : properties->getNextNamespace());
    SAFE_DELETE(properties);

    return collisionObject;
}

PhysicsCollisionObject* Node::setCollisionObject(Properties* properties)
{
    SAFE_DELETE(_collisionObject);

    // Check if the properties is valid.
    if (!properties || !(strcmp(properties->getNamespace(), "collisionObject") == 0))
    {
        GP_ERROR("Failed to load collision object from properties object: must be non-null object and have namespace equal to 'collisionObject'.");
        return NULL;
    }

    if (const char* type = properties->getString("type"))
    {
        if (strcmp(type, "CHARACTER") == 0)
        {
            _collisionObject = PhysicsCharacter::create(this, properties);
        }
        else if (strcmp(type, "GHOST_OBJECT") == 0)
        {
            _collisionObject = PhysicsGhostObject::create(this, properties);
        }
        else if (strcmp(type, "RIGID_BODY") == 0)
        {
            _collisionObject = PhysicsRigidBody::create(this, properties);
        }
        else if (strcmp(type, "VEHICLE") == 0)
        {
            _collisionObject = PhysicsVehicle::create(this, properties);
        }
        else if (strcmp(type, "VEHICLE_WHEEL") == 0)
        {
            //
            // PhysicsVehicleWheel is special because this call will traverse up the scene graph for the
            // first ancestor node that is shared with another node of collision type VEHICLE, and then
            // proceed to add itself as a wheel onto that vehicle. This is by design, and allows the
            // visual scene hierarchy to be the sole representation of the relationship between physics
            // objects rather than forcing that upon the otherwise-flat ".physics" (properties) file.
            //
            // IMPORTANT: The VEHICLE must come before the VEHICLE_WHEEL in the ".scene" (properties) file!
            //
            _collisionObject = PhysicsVehicleWheel::create(this, properties);
        }
        else
        {
            GP_ERROR("Unsupported collision object type '%s'.", type);
            return NULL;
        }
    }
    else
    {
        GP_ERROR("Failed to load collision object from properties object; required attribute 'type' is missing.");
        return NULL;
    }

    return _collisionObject;
}

unsigned int Node::getNumAdvertisedDescendants() const
{
    return (unsigned int)_advertisedDescendants.size();
}

Node* Node::getAdvertisedDescendant(unsigned int i) const
{
    return _advertisedDescendants.at(i);
}

void Node::addAdvertisedDescendant(Node* node)
{
    _advertisedDescendants.push_back(node);
}

AIAgent* Node::getAgent() const
{
    return _agent;
}

void Node::setAgent(AIAgent* agent)
{
    if (agent != _agent)
    {
        if (_agent)
        {
            Game::getInstance()->getAIController()->removeAgent(_agent);
            _agent->_node = NULL;
            SAFE_RELEASE(_agent);
        }

        _agent = agent;

        if (_agent)
        {
            _agent->addRef();
            _agent->_node = this;
            Game::getInstance()->getAIController()->addAgent(_agent);
        }
    }
}

float Node::gv(Vector3 *v, int dim) {
	switch(dim) {
		case 0: return v->x;
		case 1: return v->y;
		case 2: return v->z;
	}
	return 0;
}

void Node::sv(Vector3 *v, int dim, float val) {
	switch(dim) {
		case 0: v->x = val; break;
		case 1: v->y = val; break;
		case 2: v->z = val; break;
	}
}

Node::nodeData* Node::readData(char *filename)
{
	std::auto_ptr<Stream> stream(FileSystem::open(filename));
	if (stream.get() == NULL)
	{
		GP_ERROR("Failed to open file '%s'.", filename);
		return NULL;
	}
	stream->rewind();
	
	nodeData* data = new nodeData();
	data->typeCount = 0;

	char *str, line[2048];
    float x, y, z, w;
	std::istringstream in;
	str = stream->readLine(line, 2048);
	in.str(str);
	std::string type;
	in >> type;
	data->type = type;
    str = stream->readLine(line, 2048);
    in.str(str);
    in >> x >> y >> z >> w;
    data->rotation.set(Vector3(x, y, z), (float)(w*M_PI/180.0));
    str = stream->readLine(line, 2048);
    in.str(str);
    in >> x >> y >> z;
    data->translation.set(x, y, z);
    str = stream->readLine(line, 2048);
    in.str(str);
    in >> x >> y >> z;
    data->scale.set(x, y, z);
    str = stream->readLine(line, 2048);
    int nv = atoi(str), v = 0;
    cout << nv << " vertices" << endl;
    for(int i = 0; i < nv; i++) {
    	str = stream->readLine(line, 2048);
    	in.str(str);
    	in >> x >> y >> z;
    	data->vertices.push_back(Vector3(x, y, z));
    }
    data->worldVertices.resize(nv);
    str = stream->readLine(line, 2048);
    int ne = atoi(str), e = 0;
    cout << ne << " edges" << endl;
    unsigned short v1, v2, v3;
    std::vector<unsigned short> edge(2);
    for(int i = 0; i < ne; i++) {
    	str = stream->readLine(line, 2048);
    	in.str(str);
    	in >> v1 >> v2;
    	edge[0] = v1; edge[1] = v2;
    	data->edges.push_back(edge);
    }
    str = stream->readLine(line, 2048);
    int nf = atoi(str), faceSize, numTriangles;
    std::vector<unsigned short> triangle(3), face;
    data->triangles.resize(nf);
    for(int i = 0; i < nf; i++) {
    	str = stream->readLine(line, 2048);
    	faceSize = atoi(str);
    	face.clear();
    	str = stream->readLine(line, 2048);
    	in.str(str);
    	for(int j = 0; j < faceSize; j++) {
    		in >> v1;
    		face.push_back(v1);
    	}
    	data->faces.push_back(face);
    	str = stream->readLine(line, 2048);
    	numTriangles = atoi(str);
    	for(int j = 0; j < numTriangles; j++) {
			str = stream->readLine(line, 2048);
			in.str(str);
			for(int k = 0; k < 3; k++) {
	    		in >> v1;
	    		triangle[k] = v1;
	    	}
	    	data->triangles[i].push_back(triangle);
    	}
    }
    //set the vertices according to the initial rotation, translation, & scale
/*    data->initTrans = Matrix::identity();
    data->initTrans.rotate(data->rotation);
    data->initTrans.scale(data->scale);
    for(int i = 0; i < data->vertices.size(); i++) {
    	data->initTrans.transformVector(data->vertices[i], &data->worldVertices[i]);
    	data->worldVertices[i] += data->translation;
    }//*/
    //physics
    str = stream->readLine(line, 2048);
	in.str(str);
	in >> type;
	data->objType = type;
    str = stream->readLine(line, 2048);
    int nh = atoi(str), hullSize;
    data->hulls.resize(nh);
    for(int i = 0; i < nh; i++) {
	    str = stream->readLine(line, 2048);
	    hullSize = atoi(str);
	    str = stream->readLine(line, 2048);
	    in.str(str);
    	for(int j = 0; j < hullSize; j++) {
    		in >> v1;
    		data->hulls[i].push_back(v1);
    	}
    }
    str = stream->readLine(line, 2048);
    int nc = atoi(str);
    data->constraints.resize(nc);
    std::string word;
    for(int i = 0; i < nc; i++) {
    	data->constraints[i] = new nodeConstraint();
	    str = stream->readLine(line, 2048);
	    in.str(str);
	    in >> word;
	    data->constraints[i]->type = word.c_str();
	    in >> word;
	    data->constraints[i]->other = word.c_str();
    	in >> x >> y >> z >> w;
    	data->constraints[i]->rotation.set(x, y, z, w);
    	in >> x >> y >> z;
    	data->constraints[i]->translation.set(x, y, z);
    }
    str = stream->readLine(line, 2048);
    data->mass = atof(str);
    stream->close();
    return data;
}

void Node::writeData(Node::nodeData *data, char *filename, Node *node) {
	std::auto_ptr<Stream> stream(FileSystem::open(filename, FileSystem::WRITE));
	if (stream.get() == NULL)
	{
		GP_ERROR("Failed to open file '%s'.", filename);
		return;
	}
	std::string line;
	std::ostringstream os;
	os << data->type << endl;
	Vector3 axis;
	float angle = data->rotation.toAxisAngle(&axis) * 180.0f/M_PI;
	os << axis.x << "\t" << axis.y << "\t" << axis.z << "\t" << angle << endl;
	os << data->translation.x << "\t" << data->translation.y << "\t" << data->translation.z << endl;
	os << data->scale.x << "\t" << data->scale.y << "\t" << data->scale.z << endl;
	os << data->vertices.size() << endl;
	for(int i = 0; i < data->vertices.size(); i++) {
		for(int j = 0; j < 3; j++) os << gv(&data->vertices[i],j) << "\t";
		os << endl;
	}
	line = os.str();
	stream->write(line.c_str(), sizeof(char), line.length());
	os.str("");
	os << data->edges.size() << endl;
	for(int i = 0; i < data->edges.size(); i++) {
		for(int j = 0; j < 2; j++) os << data->edges[i][j] << "\t";
		os << endl;
	}
	line = os.str();
	stream->write(line.c_str(), sizeof(char), line.length());
	os.str("");
	os << data->faces.size() << endl;
	for(int i = 0; i < data->faces.size(); i++) {
		os << data->faces[i].size() << endl;
		for(int j = 0; j < data->faces[i].size(); j++) os << data->faces[i][j] << "\t";
		os << endl << data->triangles[i].size() << endl;
		for(int j = 0; j < data->triangles[i].size(); j++) {
			for(int k = 0; k < 3; k++) os << data->triangles[i][j][k] << "\t";
			os << endl;
		}
	}
	line = os.str();
	stream->write(line.c_str(), sizeof(char), line.length());
	os.str("");
	os << data->objType << endl;
	os << data->hulls.size() << endl;
	for(int i = 0; i < data->hulls.size(); i++) {
		os << data->hulls[i].size() << endl;
		for(int j = 0; j < data->hulls[i].size(); j++) os << data->hulls[i][j] << "\t";
		os << endl;
	}
	line = os.str();
	stream->write(line.c_str(), sizeof(char), line.length());
	os.str("");
	os << data->constraints.size() << endl;
	for(int i = 0; i < data->constraints.size(); i++) {
		os << data->constraints[i]->type << "\t" << data->constraints[i]->other << "\t";
		os << data->constraints[i]->rotation.x << "\t";
		os << data->constraints[i]->rotation.y << "\t";
		os << data->constraints[i]->rotation.z << "\t";
		os << data->constraints[i]->rotation.w << "\t";
		os << data->constraints[i]->translation.x << "\t";
		os << data->constraints[i]->translation.y << "\t";
		os << data->constraints[i]->translation.z << endl;
	}
	float mass = (node != NULL) ? node->getCollisionObject()->asRigidBody()->getMass() : data->mass;
	os << mass << endl;
	line = os.str();
	stream->write(line.c_str(), sizeof(char), line.length());
	os.str("");
	stream->close();
}

void Node::writeMyData(char *filename) {
	if(filename == NULL) filename = Game::getInstance()->concat(3, "res/common/", getId(), ".node");
	writeData((nodeData*)getUserPointer(), filename, this);
}

void Node::loadData(char *filename) {
	nodeData *data = readData(filename);
	setUserPointer(data);
}

void Node::updateData() {
	nodeData *data = (nodeData*)getUserPointer();
	if(data == NULL) return;
	Matrix world = getWorldMatrix();
	Vector3 translation = getTranslationWorld();
	for(int i = 0; i < data->vertices.size(); i++) {
		world.transformVector(data->vertices[i], &data->worldVertices[i]);
		data->worldVertices[i] += translation;
	}
}

void Node::reloadFromData(char *filename, bool addPhysics) {
	loadData(filename);
	nodeData *data = (nodeData*)getUserPointer();
	std::string materialFile = Game::getInstance()->concat(2, "res/common/models.material#", data->type.c_str());
	updateData();
	//update the mesh to contain the new coordinates
	float *vertices, radius = 0;
	unsigned short *triangles;
	int numVertices = 0, numTriangles = 0, v = 0, t = 0, f = 0;
	Vector3 min(1000,1000,1000), max(-1000,-1000,-1000);
	for(int i = 0; i < data->faces.size(); i++) {
		numVertices += data->faces[i].size();
		numTriangles += data->triangles[i].size();
	}
	vertices = new float[numVertices*6];
	triangles = new unsigned short[numTriangles*3];
	std::vector<Vector3> vec(3), normal(data->faces.size());
	cout << "making node " << filename << " with " << numVertices << " vertices, " << numTriangles << " triangles, and "
		<< data->faces.size() << " faces" << endl;
	for(int i = 0; i < data->faces.size(); i++) {
		for(int j = 0; j < 3; j++) vec[j].set(data->worldVertices[data->faces[i][j]]);
		normal[i].set(vec[1] - vec[0]);
		normal[i].cross(vec[2] - vec[1]);
		normal[i].normalize(&normal[i]);
		for(int j = 0; j < data->faces[i].size(); j++) {
			for(int k = 0; k < 3; k++) {
				radius = fmaxf(radius, data->worldVertices[data->faces[i][j]].length());
				vertices[v++] = gv(&data->worldVertices[data->faces[i][j]],k);
				if(vertices[v-1] < gv(&min, k)) sv(&min, k, vertices[v-1]);
				if(vertices[v-1] > gv(&max, k)) sv(&max, k, vertices[v-1]);
			}
			for(int k = 0; k < 3; k++) vertices[v++] = gv(&normal[i],k);
		}
		for(int j = 0; j < data->triangles[i].size(); j++) {
			for(int k = 0; k < 3; k++) triangles[t++] = f + data->triangles[i][j][k];
		}
		f += data->faces[i].size();
	}
	VertexFormat::Element elements[] =
	{
		VertexFormat::Element(VertexFormat::POSITION, 3),
		VertexFormat::Element(VertexFormat::NORMAL, 3)
	};
	Mesh* mesh = Mesh::createMesh(VertexFormat(elements, 2), numVertices, false);
	mesh->setVertexData(&vertices[0], 0, numVertices);
	MeshPart *part = mesh->addPart(Mesh::TRIANGLES, Mesh::INDEX16, numTriangles*3);
	part->setIndexData(triangles, 0, numTriangles*3);
	mesh->setBoundingBox(BoundingBox(min, max));
	mesh->setBoundingSphere(BoundingSphere(Vector3::zero(), radius));
	Model *model = Model::create(mesh);
	mesh->release();
	model->setMaterial(materialFile.c_str());
	setModel(model);
	model->release();
	//set the initial transformation
	setRotation(data->rotation);
	setTranslation(data->translation);
	setScale(data->scale);
	//add the collision object
	if(addPhysics) {
		//load the compound convex hull collision object
		PhysicsRigidBody::Parameters params;
		params.mass = data->mass;
		if(data->objType.compare("mesh") == 0)
			setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::mesh(mesh), &params);
		else if(data->objType.compare("box") == 0)
			setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::box(), &params);
		else if(data->objType.compare("sphere") == 0)
			setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::sphere(), &params);
	}
	updateData();
}

NodeCloneContext::NodeCloneContext()
{
}

NodeCloneContext::~NodeCloneContext()
{
}

Animation* NodeCloneContext::findClonedAnimation(const Animation* animation)
{
    GP_ASSERT(animation);

    std::map<const Animation*, Animation*>::iterator it = _clonedAnimations.find(animation);
    return it != _clonedAnimations.end() ? it->second : NULL;
}

void NodeCloneContext::registerClonedAnimation(const Animation* original, Animation* clone)
{
    GP_ASSERT(original);
    GP_ASSERT(clone);

    _clonedAnimations[original] = clone;
}

Node* NodeCloneContext::findClonedNode(const Node* node)
{
    GP_ASSERT(node);

    std::map<const Node*, Node*>::iterator it = _clonedNodes.find(node);
    return it != _clonedNodes.end() ? it->second : NULL;
}

void NodeCloneContext::registerClonedNode(const Node* original, Node* clone)
{
    GP_ASSERT(original);
    GP_ASSERT(clone);

    _clonedNodes[original] = clone;
}

}
