# Quickstart Guide: AI-Native Textbook System

**Feature**: AI-Native Textbook System
**Date**: 2025-12-07
**Guide Version**: 1.0

## Overview

This quickstart guide will help you get the AI-Native Textbook System up and running for development. The system consists of a frontend application (Docusaurus-based textbook site) and a backend API (FastAPI) with AI tutoring capabilities.

## Prerequisites

Before starting, ensure you have the following installed:

### System Requirements

- **Operating System**: Linux, macOS, or Windows with WSL2
- **Node.js**: v18.x or higher
- **Python**: v3.11 or higher
- **Docker**: v20.x or higher (for containerized services)
- **Git**: v2.30 or higher

### External Services (for full functionality)

- **Qdrant Cloud**: Vector database for RAG system (free tier available)
- **Neon Postgres**: Serverless Postgres database (free tier available)
- **OpenAI API Key**: For AI tutoring capabilities
- **Better-Auth compatible email service**: For authentication flows

## Setting Up the Development Environment

### 1. Clone the Repository

```bash
git clone https://github.com/your-org/ai-native-book.git
cd ai-native-book
```

### 2. Install Backend Dependencies

```bash
# Navigate to backend directory (create if doesn't exist)
mkdir -p backend
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install fastapi uvicorn python-multipart pydantic sqlalchemy psycopg2-binary openai python-jose[cryptography] passlib[bcrypt] qdrant-client

# Or if you have a requirements.txt file:
pip install -r requirements.txt
```

### 3. Install Frontend Dependencies

```bash
# Navigate to frontend directory (root of project for Docusaurus)
cd ..

# Install Node.js dependencies
npm install

# If using Yarn:
yarn install
```

### 4. Configure Environment Variables

Create `.env` files for both backend and frontend:

#### Backend `.env` file

# Database

DATABASE_URL=postgresql://username:password@localhost:5432/ai_textbook

# Qdrant Vector Database

QDRANT_URL=https://your-cluster-url.qdrant.io
QDRANT_API_KEY=your-api-key
QDRANT_COLLECTION_NAME=textbook_content

# OpenAI

OPENAI_API_KEY=your-openai-api-key

# Authentication

SECRET_KEY=your-secret-key-here-make-it-long-and-random
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

# Application

API_PREFIX=/api
DEBUG=true
```

#### Frontend `.env` file:
```
# Backend API
REACT_APP_API_URL=http://localhost:8000
REACT_APP_AUTH_URL=http://localhost:8000/auth

# Qdrant (if direct access needed)
REACT_APP_QDRANT_URL=your-qdrant-url

# Feature flags
REACT_APP_ENABLE_OFFLINE_MODE=true
REACT_APP_ENABLE_TRANSLATION=true
```

### 5. Set Up Database

#### For PostgreSQL (Neon):

1. Sign up at [Neon](https://neon.tech) and create a new project
2. Get the connection string from the project dashboard
3. Update your `DATABASE_URL` in the backend `.env` file

#### Run database migrations:
```bash
# In backend directory with virtual environment activated
cd backend
python -m alembic upgrade head
```

### 6. Set Up Vector Database (Qdrant)

1. Sign up at [Qdrant Cloud](https://qdrant.tech) or run locally:
```bash
docker run -p 6333:6333 -v $(pwd)/qdrant_storage:/qdrant/storage:z qdrant/qdrant
```

2. Create a collection for the textbook content:
```bash
curl -X PUT 'http://localhost:6333/collections/textbook_content' \
-H 'Content-Type: application/json' \
--data-raw '{
  "vector_size": 1536,
  "distance": "Cosine"
}'
```

## Running the Applications

### 1. Start the Backend API

```bash
# In backend directory with virtual environment activated
cd backend
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

The backend API will be available at `http://localhost:8000`.

API documentation will be available at:
- `http://localhost:8000/docs` - Interactive API documentation (Swagger UI)
- `http://localhost:8000/redoc` - Alternative API documentation (ReDoc)

### 2. Start the Frontend Application

```bash
# In project root directory
cd ..
npm start
```

The frontend application will be available at `http://localhost:3000`.

### 3. Running Both Simultaneously

You can use a process manager like `concurrently`:

```bash
npm install -g concurrently
concurrently "cd backend && uvicorn src.main:app --reload --host 0.0.0.0 --port 8000" "cd . && npm start"
```

## Key Endpoints and Interfaces

### Backend API Endpoints

#### User Authentication
- `POST /auth/register` - User registration
- `POST /auth/login` - User login
- `GET /auth/profile` - Get user profile (requires auth)

#### Textbook Content
- `GET /chapters/` - List all textbook chapters
- `GET /chapters/{chapter_id}` - Get specific chapter content
- `GET /chapters/{chapter_id}/sections` - Get sections within a chapter

#### AI Tutor
- `POST /ai/tutor/ask` - Ask the AI tutor a question
- `POST /ai/tutor/feedback` - Provide feedback on AI tutor response

#### User Progress
- `GET /progress/{user_id}/chapters` - Get user progress across chapters
- `POST /progress/{user_id}/chapter/{chapter_id}/update` - Update progress for a chapter

### Frontend Pages

- `/` - Home page with textbook overview
- `/chapters` - List of all textbook chapters
- `/chapters/:id` - Individual chapter view
- `/auth/login` - Login page
- `/auth/register` - Registration page
- `/dashboard` - User dashboard with progress tracking
- `/ai-tutor` - Dedicated AI tutor interface

## Development Workflow

### Adding New Chapters

1. Create the chapter content in the `/docs` directory following Docusaurus standards
2. Add the chapter to the sidebar configuration in `sidebars.js`
3. Create corresponding API endpoints in the backend to retrieve additional data (quizzes, code examples, etc.)
4. Update the data model if needed to support new content types

### Implementing New AI Capabilities

1. Add new endpoints to the AI service in the backend
2. Update the AIKnowledgeBase model if new content types need to be indexed
3. Create or update RAG retrieval functions
4. Add corresponding UI components in the frontend

### Testing

#### Backend Tests
```bash
cd backend
python -m pytest
```

#### Frontend Tests
```bash
npm test
```

#### End-to-End Tests (when available)
```bash
npm run e2e
```

## API Documentation

Interactive API documentation is available at `http://localhost:8000/docs` when the backend is running.

## Troubleshooting

### Common Issues

1. **Port Conflicts**: If ports 8000 or 3000 are already in use, change them in the startup commands.

2. **Environment Variables Not Loaded**: Make sure `.env` files are in the correct directories and the application is restarted after changes.

3. **Database Connection Issues**: Verify the `DATABASE_URL` is correct and your database instance is running and accessible.

4. **Qdrant Connection Issues**: Ensure Qdrant is running and the URL/credentials in your environment are correct.

### Resetting Development Data

To reset development data:
```bash
# Reset database (run with caution!)
cd backend
python reset_db.py

# Clear local Qdrant collection
curl -X DELETE 'http://localhost:6333/collections/textbook_content'
```

## Next Steps

1. Follow the detailed development guides in the `/docs` directory for:
   - Adding new textbook content
   - Extending AI tutoring capabilities
   - Implementing new personalization features
   - Adding multi-language support

2. Review the architecture documentation in `specs/002-ai-native-textbook/` for:
   - Implementation details
   - API contracts
   - Data models

3. Join the development team's communication channels for support and collaboration.
