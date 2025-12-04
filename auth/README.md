# Authentication Service

Better Auth-based authentication service for the Physical AI Textbook platform.

## Features

- ✅ Email/Password authentication
- ✅ Custom user field: `hardware_bg` (RTX4090, Jetson, Laptop, Cloud)
- ✅ Neon Postgres integration
- ✅ Session management (7-day expiry)
- ✅ CORS-enabled for frontend integration
- ✅ Health check endpoints

## Quick Start

### 1. Install Dependencies

```bash
cd auth
npm install
```

### 2. Configure Environment

Make sure your `.env` file in the project root contains:

```bash
# Neon Postgres
DATABASE_URL=postgresql://user:password@host.neon.tech:5432/database?sslmode=require

# Auth configuration
AUTH_SECRET=your_random_secret_key_at_least_32_characters_long
AUTH_URL=http://localhost:3001
AUTH_PORT=3001

# CORS
CORS_ORIGINS=http://localhost:3000,http://localhost:8000
```

**Generate AUTH_SECRET**:
```bash
node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"
```

### 3. Run Database Migration

```bash
npm run migrate:dev
```

This will:
- Create the `users` table
- Add the custom `hardware_bg` column
- Set up Better Auth schema

### 4. Start the Server

```bash
# Development mode (with hot reload)
npm run dev

# Production mode
npm start
```

Server will start on: **http://localhost:3001**

## API Endpoints

### Core Authentication (Better Auth)

All Better Auth endpoints are available at `/api/auth/*`:

#### Sign Up
```bash
POST /api/auth/sign-up
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "securepassword123",
  "name": "John Doe"
}
```

#### Sign In
```bash
POST /api/auth/sign-in
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "securepassword123"
}
```

**Response**:
```json
{
  "user": {
    "id": "uuid",
    "email": "user@example.com",
    "name": "John Doe",
    "hardware_bg": "Laptop"
  },
  "session": {
    "token": "session_token",
    "expiresAt": "2025-12-10T00:00:00.000Z"
  }
}
```

#### Sign Out
```bash
POST /api/auth/sign-out
Authorization: Bearer <session_token>
```

#### Get Session
```bash
GET /api/auth/session
Authorization: Bearer <session_token>
```

### Custom Endpoints

#### Update Hardware Background
```bash
POST /api/user/hardware
Authorization: Bearer <session_token>
Content-Type: application/json

{
  "hardware_bg": "RTX4090"
}
```

**Valid values**: `RTX4090`, `Jetson`, `Laptop`, `Cloud`

#### Get User Profile
```bash
GET /api/user/profile
Authorization: Bearer <session_token>
```

**Response**:
```json
{
  "id": "uuid",
  "email": "user@example.com",
  "name": "John Doe",
  "hardware_bg": "RTX4090",
  "createdAt": "2025-12-03T00:00:00.000Z"
}
```

### Health Check

```bash
GET /health
```

**Response**:
```json
{
  "status": "healthy",
  "services": {
    "api": "running",
    "database": "connected",
    "auth": "configured"
  },
  "database": {
    "connected": true,
    "timestamp": "2025-12-03T18:00:00.000Z"
  }
}
```

## Architecture

### User Schema

```typescript
interface User {
  id: string;                    // UUID
  email: string;                 // Unique, required
  name?: string;                 // Optional
  emailVerified: boolean;        // Email verification status
  createdAt: Date;              // Account creation timestamp
  updatedAt: Date;              // Last update timestamp

  // Custom field
  hardware_bg?: HardwareBackground; // Default: "Laptop"
}

enum HardwareBackground {
  RTX4090 = "RTX4090",   // High-end workstation
  Jetson = "Jetson",     // NVIDIA Jetson edge device
  Laptop = "Laptop",     // Standard laptop
  Cloud = "Cloud",       // Cloud-based development
}
```

### Session Configuration

- **Expiry**: 7 days
- **Update frequency**: Every 24 hours
- **Cookie prefix**: `textbook`
- **Cache**: 5-minute cookie cache

### Database Connection

- **Provider**: PostgreSQL (Neon)
- **SSL**: Required (configured automatically)
- **Connection pooling**: Enabled
- **Health checks**: Automatic on startup

## Integration with Frontend

### React Example

```typescript
// Sign up
const signUp = async (email: string, password: string) => {
  const response = await fetch('http://localhost:3001/api/auth/sign-up', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email, password })
  });
  return response.json();
};

// Sign in
const signIn = async (email: string, password: string) => {
  const response = await fetch('http://localhost:3001/api/auth/sign-in', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    credentials: 'include', // Important for cookies
    body: JSON.stringify({ email, password })
  });
  return response.json();
};

// Update hardware
const updateHardware = async (hardware_bg: string, token: string) => {
  const response = await fetch('http://localhost:3001/api/user/hardware', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${token}`
    },
    body: JSON.stringify({ hardware_bg })
  });
  return response.json();
};
```

## Database Schema (Auto-generated)

When you run `npm run migrate:dev`, Better Auth creates:

```sql
-- Users table
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  name VARCHAR(255),
  email_verified BOOLEAN DEFAULT FALSE,
  password_hash TEXT NOT NULL,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW(),

  -- Custom field
  hardware_bg VARCHAR(50) DEFAULT 'Laptop'
);

-- Sessions table
CREATE TABLE sessions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id) ON DELETE CASCADE,
  token TEXT UNIQUE NOT NULL,
  expires_at TIMESTAMP NOT NULL,
  created_at TIMESTAMP DEFAULT NOW()
);

-- Indexes
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_sessions_token ON sessions(token);
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
```

## Testing

### Manual Testing

1. **Sign up a user**:
```bash
curl -X POST http://localhost:3001/api/auth/sign-up \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"password123","name":"Test User"}'
```

2. **Sign in**:
```bash
curl -X POST http://localhost:3001/api/auth/sign-in \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"password123"}'
```

3. **Update hardware** (use token from sign-in response):
```bash
curl -X POST http://localhost:3001/api/user/hardware \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <token>" \
  -d '{"hardware_bg":"RTX4090"}'
```

### Automated Testing (TODO)

```bash
npm test
```

## Troubleshooting

### Error: "DATABASE_URL is not set"

**Solution**: Make sure `.env` file exists in project root with DATABASE_URL configured.

### Error: "Connection refused"

**Solution**: Check that Neon Postgres is accessible. Test connection:
```bash
psql "postgresql://user:password@host.neon.tech:5432/database?sslmode=require"
```

### Error: "AUTH_SECRET is not set"

**Solution**: Generate a secret and add to `.env`:
```bash
node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"
```

### Migration fails

**Solution**: Check database permissions and connection:
```bash
# Test connection
npm run migrate:dev -- --verbose
```

## Security Considerations

### Production Checklist

- [ ] Set `emailAndPassword.requireEmailVerification` to `true`
- [ ] Implement email sending (Resend, SendGrid, etc.)
- [ ] Enable HTTPS (AUTH_URL should use https://)
- [ ] Set strong AUTH_SECRET (min 32 characters)
- [ ] Configure proper CORS origins (no wildcards)
- [ ] Enable rate limiting (express-rate-limit)
- [ ] Add CSRF protection
- [ ] Implement password strength requirements
- [ ] Add account lockout after failed attempts
- [ ] Enable audit logging

### Password Requirements

Current: Minimum 8 characters (Better Auth default)

**Recommended for production**:
- Minimum 12 characters
- At least one uppercase letter
- At least one lowercase letter
- At least one number
- At least one special character

## Development

### Project Structure

```
auth/
├── server.ts           # Express server entry point
├── auth.config.ts      # Better Auth configuration
├── package.json        # Dependencies
├── tsconfig.json       # TypeScript config
├── .env.example        # Environment template
└── README.md          # This file
```

### Tech Stack

- **Runtime**: Node.js 20+
- **Language**: TypeScript
- **Framework**: Express
- **Auth**: Better Auth 1.0+
- **Database**: PostgreSQL (Neon)
- **ORM**: Better Auth's built-in Prisma integration

## Next Steps

- [ ] Implement email verification
- [ ] Add OAuth providers (GitHub, Google)
- [ ] Add password reset flow with email
- [ ] Implement rate limiting
- [ ] Add 2FA/MFA support
- [ ] Create admin dashboard
- [ ] Add user roles and permissions

## License

Part of the Physical AI & Humanoid Robotics Textbook project.
